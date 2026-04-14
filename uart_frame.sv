// =============================================================================
// Module  : uart_frame
// Desc    : UART RX/TX 115200 8N1 + Frame Parser + Debug Log
// Fixed   : Gowin EDA compatible syntax (no complex function, no local var in always)
// =============================================================================
module uart_frame #(
    parameter integer CLK_MHZ = 27,
    parameter integer BAUD    = 115200
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        rx,
    output reg         tx,

    output reg  [7:0]  reg_addr,
    output reg  [31:0] reg_wdata,
    output reg         reg_wen,
    input  wire [31:0] reg_rdata,
    output reg         reg_ren,

    output reg         uart_kick,

    input  wire        en_effective,
    input  wire        fault_active,
    input  wire        enout,
    input  wire        wdi_kick
);

// ════════════════════════════════════════════════════════════════════
// Parameters
// ════════════════════════════════════════════════════════════════════
localparam integer BIT_CYC  = CLK_MHZ * 1000000 / BAUD; // 234
localparam integer HALF_CYC = BIT_CYC / 2;               // 117
localparam integer CNT_W    = 8; // 234 < 256, dùng 8 bit cho đơn giản

// ════════════════════════════════════════════════════════════════════
// 1. UART RX
// ════════════════════════════════════════════════════════════════════
reg rx_s1, rx_s2;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin rx_s1<=1; rx_s2<=1; end
    else        begin rx_s1<=rx; rx_s2<=rx_s1; end
end

reg [1:0]  rx_state;
reg [CNT_W-1:0] rx_cnt;
reg [2:0]  rx_bit;
reg [7:0]  rx_shift;
reg [7:0]  rx_data;
reg        rx_valid;

localparam RX_IDLE=0, RX_START=1, RX_DATA=2, RX_STOP=3;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_state<=RX_IDLE; rx_cnt<='0;
        rx_bit<='0; rx_shift<='0;
        rx_data<='0; rx_valid<=0;
    end else begin
        rx_valid <= 0;
        case (rx_state)
            RX_IDLE:  if (!rx_s2) begin rx_state<=RX_START; rx_cnt<=0; end

            RX_START: begin
                if (rx_cnt == HALF_CYC-1) begin
                    rx_cnt <= 0;
                    if (!rx_s2) begin rx_state<=RX_DATA; rx_bit<=0; end
                    else         rx_state<=RX_IDLE;
                end else rx_cnt<=rx_cnt+1;
            end

            RX_DATA: begin
                if (rx_cnt == BIT_CYC-1) begin
                    rx_cnt  <= 0;
                    rx_shift <= {rx_s2, rx_shift[7:1]}; // LSB first
                    if (rx_bit==7) rx_state<=RX_STOP;
                    else rx_bit<=rx_bit+1;
                end else rx_cnt<=rx_cnt+1;
            end

            RX_STOP: begin
                if (rx_cnt == BIT_CYC-1) begin
                    rx_cnt  <= 0;
                    rx_state <= RX_IDLE;
                    if (rx_s2) begin rx_data<=rx_shift; rx_valid<=1; end
                end else rx_cnt<=rx_cnt+1;
            end
        endcase
    end
end

// ════════════════════════════════════════════════════════════════════
// 2. TX Engine (shared)
// ════════════════════════════════════════════════════════════════════
reg [7:0]        tx_byte;
reg              tx_send;
reg              tx_busy;
reg [CNT_W-1:0] tx_cnt;
reg [9:0]        tx_shift; // stop + data[7:0] + start
reg [3:0]        tx_bits;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tx<=1; tx_cnt<=0; tx_bits<=0;
        tx_shift<=10'h3FF; tx_busy<=0;
    end else begin
        if (!tx_busy && tx_send) begin
            tx_shift <= {1'b1, tx_byte, 1'b0}; // [stop|data|start]
            tx_cnt   <= 0;
            tx_bits  <= 0;
            tx_busy  <= 1;
            tx       <= 0; // start bit
        end else if (tx_busy) begin
            if (tx_cnt == BIT_CYC-1) begin
                tx_cnt  <= 0;
                tx_bits <= tx_bits+1;
                tx_shift<= {1'b1, tx_shift[9:1]};
                tx      <= tx_shift[1];
                if (tx_bits==9) begin tx_busy<=0; tx<=1; end
            end else tx_cnt<=tx_cnt+1;
        end
    end
end

// ════════════════════════════════════════════════════════════════════
// 3. TX Arbiter: frame_resp (ưu tiên) vs debug_log
// ════════════════════════════════════════════════════════════════════
reg [7:0] frame_byte; reg frame_req;
reg [7:0] debug_byte; reg debug_req;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tx_byte<=0; tx_send<=0;
    end else begin
        tx_send <= 0;
        if (!tx_busy) begin
            if (frame_req) begin
                tx_byte <= frame_byte;
                tx_send <= 1;
            end else if (debug_req) begin
                tx_byte <= debug_byte;
                tx_send <= 1;
            end
        end
    end
end

// ════════════════════════════════════════════════════════════════════
// 4. Frame Parser
// ════════════════════════════════════════════════════════════════════
reg [2:0]  fp_state;
reg [7:0]  fp_cmd, fp_addr_r, fp_len;
reg [7:0]  fp_d0, fp_d1, fp_d2, fp_d3;
reg [7:0]  fp_chk;
reg [1:0]  fp_didx;

localparam FP_IDLE=0,FP_CMD=1,FP_ADDR=2,FP_LEN=3,FP_DATA=4,FP_CHK=5;

// Response buffer
reg [7:0]  resp0,resp1,resp2,resp3,resp4,resp5,resp6;
reg [2:0]  resp_total, resp_idx;
reg        resp_busy;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        fp_state<=FP_IDLE; fp_chk<=0;
        fp_didx<=0;
        reg_wen<=0; reg_ren<=0; uart_kick<=0;
        reg_addr<=0; reg_wdata<=0;
        resp_busy<=0; resp_idx<=0; resp_total<=0;
        frame_req<=0; frame_byte<=0;
        fp_cmd<=0; fp_addr_r<=0; fp_len<=0;
        fp_d0<=0; fp_d1<=0; fp_d2<=0; fp_d3<=0;
    end else begin
        reg_wen   <= 0;
        reg_ren   <= 0;
        uart_kick <= 0;
        frame_req <= 0;

        // ── Frame RX ──
        if (rx_valid) begin
            case (fp_state)
                FP_IDLE: if (rx_data==8'h55) fp_state<=FP_CMD;

                FP_CMD: begin
                    fp_cmd   <= rx_data;
                    fp_chk   <= rx_data;
                    fp_state <= FP_ADDR;
                end

                FP_ADDR: begin
                    fp_addr_r<= rx_data;
                    fp_chk   <= fp_chk ^ rx_data;
                    fp_state <= FP_LEN;
                end

                FP_LEN: begin
                    fp_len   <= rx_data;
                    fp_chk   <= fp_chk ^ rx_data;
                    fp_didx  <= 0;
                    if (rx_data==0) fp_state<=FP_CHK;
                    else            fp_state<=FP_DATA;
                end

                FP_DATA: begin
                    fp_chk <= fp_chk ^ rx_data;
                    case (fp_didx)
                        0: fp_d0<=rx_data;
                        1: fp_d1<=rx_data;
                        2: fp_d2<=rx_data;
                        3: fp_d3<=rx_data;
                    endcase
                    if (fp_didx==fp_len[1:0]-1) fp_state<=FP_CHK;
                    else fp_didx<=fp_didx+1;
                end

                FP_CHK: begin
                    fp_state <= FP_IDLE;
                    if (rx_data==fp_chk) begin
                        // Checksum OK
                        case (fp_cmd)
                            8'h01: begin // WRITE_REG
                                reg_addr  <= fp_addr_r;
                                reg_wdata <= {fp_d3,fp_d2,fp_d1,fp_d0};
                                reg_wen   <= 1;
                                resp0<=8'h55; resp1<=8'hAA; resp2<=8'hAA;
                                resp_total<=3;
                            end
                            8'h02: begin // READ_REG
                                reg_addr  <= fp_addr_r;
                                reg_ren   <= 1;
                                resp_total<= 0; // built after rdata
                            end
                            8'h03: begin // KICK
                                uart_kick <= 1;
                                resp0<=8'h55; resp1<=8'hAA; resp2<=8'hAA;
                                resp_total<=3;
                            end
                            8'h04: begin // GET_STATUS
                                reg_addr  <= 8'h10;
                                reg_ren   <= 1;
                                resp_total<= 0;
                            end
                            default: begin
                                resp0<=8'h55; resp1<=8'hFF; resp2<=8'hFF;
                                resp_total<=3;
                            end
                        endcase
                    end else begin
                        // NACK
                        resp0<=8'h55; resp1<=8'hFF; resp2<=8'hFF;
                        resp_total<=3;
                    end
                    if (!resp_busy && resp_total>0) begin
                        resp_idx  <= 0;
                        resp_busy <= 1;
                    end
                end
            endcase
        end

        // ── Build READ response 1 cycle sau reg_ren ──
        if (reg_ren) begin
            // sẽ có rdata ở cycle tiếp theo
        end

        // ── Send response bytes ──
        if (resp_busy && !tx_busy && !frame_req) begin
            case (resp_idx)
                0: frame_byte<=resp0;
                1: frame_byte<=resp1;
                2: frame_byte<=resp2;
                3: frame_byte<=resp3;
                4: frame_byte<=resp4;
                5: frame_byte<=resp5;
                6: frame_byte<=resp6;
                default: frame_byte<=0;
            endcase
            frame_req <= 1;
            if (resp_idx==resp_total-1) resp_busy<=0;
            else resp_idx<=resp_idx+1;
        end
    end
end

// Build READ response sau 1 cycle
reg reg_ren_d;
reg [31:0] rdata_latch;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        reg_ren_d<=0; rdata_latch<=0;
    end else begin
        reg_ren_d   <= reg_ren;
        if (reg_ren) rdata_latch <= reg_rdata;
        if (reg_ren_d && !resp_busy) begin
            resp0 <= 8'h55;
            resp1 <= 8'hAA;
            resp2 <= rdata_latch[7:0];
            resp3 <= rdata_latch[15:8];
            resp4 <= rdata_latch[23:16];
            resp5 <= rdata_latch[31:24];
            resp6 <= 8'hAA ^ rdata_latch[7:0] ^ rdata_latch[15:8]
                           ^ rdata_latch[23:16] ^ rdata_latch[31:24];
        end
    end
end

// ════════════════════════════════════════════════════════════════════
// 5. Debug Log — ROM-based, LSB first (đã fix)
// ════════════════════════════════════════════════════════════════════
// Message ROM: 7 messages, mỗi message tối đa 24 bytes
// Index: 0=BOOT, 1=DISABLED, 2=ARMING, 3=RUNNING, 4=KICK, 5=TIMEOUT, 6=RECOVER
reg [7:0] msg_rom [0:6][0:23];
reg [4:0] msg_len_rom [0:6];

initial begin
    // 0: "=== WDG READY ===\r\n" (19)
    msg_rom[0][0]="="; msg_rom[0][1]="="; msg_rom[0][2]="=";
    msg_rom[0][3]=" "; msg_rom[0][4]="W"; msg_rom[0][5]="D";
    msg_rom[0][6]="G"; msg_rom[0][7]=" "; msg_rom[0][8]="R";
    msg_rom[0][9]="E"; msg_rom[0][10]="A"; msg_rom[0][11]="D";
    msg_rom[0][12]="Y"; msg_rom[0][13]=" "; msg_rom[0][14]="=";
    msg_rom[0][15]="="; msg_rom[0][16]="="; msg_rom[0][17]=8'h0D;
    msg_rom[0][18]=8'h0A; msg_len_rom[0]=19;

    // 1: "[WDG] DISABLED\r\n" (16)
    msg_rom[1][0]="["; msg_rom[1][1]="W"; msg_rom[1][2]="D";
    msg_rom[1][3]="G"; msg_rom[1][4]="]"; msg_rom[1][5]=" ";
    msg_rom[1][6]="D"; msg_rom[1][7]="I"; msg_rom[1][8]="S";
    msg_rom[1][9]="A"; msg_rom[1][10]="B"; msg_rom[1][11]="L";
    msg_rom[1][12]="E"; msg_rom[1][13]="D"; msg_rom[1][14]=8'h0D;
    msg_rom[1][15]=8'h0A; msg_len_rom[1]=16;

    // 2: "[WDG] ARMING...\r\n" (17)
    msg_rom[2][0]="["; msg_rom[2][1]="W"; msg_rom[2][2]="D";
    msg_rom[2][3]="G"; msg_rom[2][4]="]"; msg_rom[2][5]=" ";
    msg_rom[2][6]="A"; msg_rom[2][7]="R"; msg_rom[2][8]="M";
    msg_rom[2][9]="I"; msg_rom[2][10]="N"; msg_rom[2][11]="G";
    msg_rom[2][12]="."; msg_rom[2][13]="."; msg_rom[2][14]=".";
    msg_rom[2][15]=8'h0D; msg_rom[2][16]=8'h0A; msg_len_rom[2]=17;

    // 3: "[WDG] RUNNING\r\n" (15)
    msg_rom[3][0]="["; msg_rom[3][1]="W"; msg_rom[3][2]="D";
    msg_rom[3][3]="G"; msg_rom[3][4]="]"; msg_rom[3][5]=" ";
    msg_rom[3][6]="R"; msg_rom[3][7]="U"; msg_rom[3][8]="N";
    msg_rom[3][9]="N"; msg_rom[3][10]="I"; msg_rom[3][11]="N";
    msg_rom[3][12]="G"; msg_rom[3][13]=8'h0D; msg_rom[3][14]=8'h0A;
    msg_len_rom[3]=15;

    // 4: "[WDG] KICK!\r\n" (13)
    msg_rom[4][0]="["; msg_rom[4][1]="W"; msg_rom[4][2]="D";
    msg_rom[4][3]="G"; msg_rom[4][4]="]"; msg_rom[4][5]=" ";
    msg_rom[4][6]="K"; msg_rom[4][7]="I"; msg_rom[4][8]="C";
    msg_rom[4][9]="K"; msg_rom[4][10]="!"; msg_rom[4][11]=8'h0D;
    msg_rom[4][12]=8'h0A; msg_len_rom[4]=13;

    // 5: "[WDG] TIMEOUT!\r\n" (16)
    msg_rom[5][0]="["; msg_rom[5][1]="W"; msg_rom[5][2]="D";
    msg_rom[5][3]="G"; msg_rom[5][4]="]"; msg_rom[5][5]=" ";
    msg_rom[5][6]="T"; msg_rom[5][7]="I"; msg_rom[5][8]="M";
    msg_rom[5][9]="E"; msg_rom[5][10]="O"; msg_rom[5][11]="U";
    msg_rom[5][12]="T"; msg_rom[5][13]="!"; msg_rom[5][14]=8'h0D;
    msg_rom[5][15]=8'h0A; msg_len_rom[5]=16;

    // 6: "[WDG] RECOVER\r\n" (15)
    msg_rom[6][0]="["; msg_rom[6][1]="W"; msg_rom[6][2]="D";
    msg_rom[6][3]="G"; msg_rom[6][4]="]"; msg_rom[6][5]=" ";
    msg_rom[6][6]="R"; msg_rom[6][7]="E"; msg_rom[6][8]="C";
    msg_rom[6][9]="O"; msg_rom[6][10]="V"; msg_rom[6][11]="E";
    msg_rom[6][12]="R"; msg_rom[6][13]=8'h0D; msg_rom[6][14]=8'h0A;
    msg_len_rom[6]=15;
end

// State tracking
reg en_prev_d, fault_prev_d, enout_prev_d, kick_prev_d;
reg boot_sent;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        en_prev_d<=0; fault_prev_d<=0;
        enout_prev_d<=0; kick_prev_d<=0;
    end else begin
        en_prev_d    <= en_effective;
        fault_prev_d <= fault_active;
        enout_prev_d <= enout;
        kick_prev_d  <= wdi_kick;
    end
end

// Debug TX state machine
reg [1:0]  dbg_state;
reg [2:0]  dbg_msg;
reg [4:0]  dbg_idx;

localparam DBG_IDLE=0, DBG_SEND=1;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        dbg_state<=DBG_IDLE; dbg_msg<=0;
        dbg_idx<=0; debug_req<=0; debug_byte<=0;
        boot_sent<=0;
    end else begin
        debug_req <= 0;

        case (dbg_state)
            DBG_IDLE: begin
                if (!boot_sent && !resp_busy && !tx_busy) begin
                    dbg_msg<=0; dbg_idx<=0;
                    dbg_state<=DBG_SEND; boot_sent<=1;
                end else if (!resp_busy) begin
                    if      (fault_active && !fault_prev_d)
                        begin dbg_msg<=5; dbg_idx<=0; dbg_state<=DBG_SEND; end
                    else if (!fault_active && fault_prev_d && en_effective)
                        begin dbg_msg<=6; dbg_idx<=0; dbg_state<=DBG_SEND; end
                    else if (wdi_kick && !kick_prev_d)
                        begin dbg_msg<=4; dbg_idx<=0; dbg_state<=DBG_SEND; end
                    else if (enout && !enout_prev_d)
                        begin dbg_msg<=3; dbg_idx<=0; dbg_state<=DBG_SEND; end
                    else if (en_effective && !en_prev_d && !enout)
                        begin dbg_msg<=2; dbg_idx<=0; dbg_state<=DBG_SEND; end
                    else if (!en_effective && en_prev_d)
                        begin dbg_msg<=1; dbg_idx<=0; dbg_state<=DBG_SEND; end
                end
            end

            DBG_SEND: begin
                if (!tx_busy && !frame_req) begin
                    debug_byte <= msg_rom[dbg_msg][dbg_idx];
                    debug_req  <= 1;
                    if (dbg_idx == msg_len_rom[dbg_msg]-1)
                        dbg_state <= DBG_IDLE;
                    else
                        dbg_idx <= dbg_idx+1;
                end
            end

            default: dbg_state<=DBG_IDLE;
        endcase
    end
end

endmodule
