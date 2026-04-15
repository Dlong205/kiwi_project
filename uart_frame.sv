// =============================================================================
// Module  : uart_frame
// Desc    : UART RX/TX 115200 8N1 + Frame Parser + Debug Log
// Fixed   : Multiple drivers, width truncation, Gowin compatible
// =============================================================================
/*
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
// Parameters — dùng đủ bit tránh truncation
// 234 cần 8 bit (max 255 OK), nhưng dùng 9 bit cho chắc
// ════════════════════════════════════════════════════════════════════
// Dùng hằng số trực tiếp tránh overflow khi nhân
localparam integer BIT_CYC  = (CLK_MHZ * 1000000) / BAUD; // 234
localparam integer HALF_CYC = BIT_CYC / 2;                 // 117

// ════════════════════════════════════════════════════════════════════
// 1. UART RX
// ════════════════════════════════════════════════════════════════════
reg rx_s1, rx_s2;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin rx_s1<=1'b1; rx_s2<=1'b1; end
    else        begin rx_s1<=rx; rx_s2<=rx_s1; end
end

localparam RX_IDLE=2'd0, RX_START=2'd1, RX_DATA=2'd2, RX_STOP=2'd3;
reg [1:0]   rx_state;
reg [8:0]   rx_cnt;     // 9 bit: max 234
reg [3:0]   rx_bit;     // 4 bit: 0..9
reg [7:0]   rx_shift;
reg [7:0]   rx_data;
reg         rx_valid;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_state<=RX_IDLE; rx_cnt<=9'd0;
        rx_bit<=4'd0; rx_shift<=8'd0;
        rx_data<=8'd0; rx_valid<=1'b0;
    end else begin
        rx_valid <= 1'b0;
        case (rx_state)
            RX_IDLE: begin
                if (!rx_s2) begin
                    rx_state <= RX_START;
                    rx_cnt   <= 9'd0;
                end
            end
            RX_START: begin
                if (rx_cnt == 9'(HALF_CYC-1)) begin
                    rx_cnt <= 9'd0;
                    if (!rx_s2) begin rx_state<=RX_DATA; rx_bit<=4'd0; end
                    else         rx_state<=RX_IDLE;
                end else rx_cnt<=rx_cnt+9'd1;
            end
            RX_DATA: begin
                if (rx_cnt == 9'(BIT_CYC-1)) begin
                    rx_cnt   <= 9'd0;
                    rx_shift <= {rx_s2, rx_shift[7:1]};
                    if (rx_bit==4'd7) rx_state<=RX_STOP;
                    else rx_bit<=rx_bit+4'd1;
                end else rx_cnt<=rx_cnt+9'd1;
            end
            RX_STOP: begin
                if (rx_cnt == 9'(BIT_CYC-1)) begin
                    rx_cnt   <= 9'd0;
                    rx_state <= RX_IDLE;
                    if (rx_s2) begin rx_data<=rx_shift; rx_valid<=1'b1; end
                end else rx_cnt<=rx_cnt+9'd1;
            end
            default: rx_state<=RX_IDLE;
        endcase
    end
end

// ════════════════════════════════════════════════════════════════════
// 2. TX Engine
// ════════════════════════════════════════════════════════════════════
reg [7:0]  tx_byte;
reg        tx_send;
reg        tx_busy;
reg [8:0]  tx_cnt;      // 9 bit
reg [9:0]  tx_shift;
reg [3:0]  tx_bits;     // 4 bit: 0..9

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tx<=1'b1; tx_cnt<=9'd0; tx_bits<=4'd0;
        tx_shift<=10'h3FF; tx_busy<=1'b0;
    end else begin
        if (!tx_busy && tx_send) begin
            tx_shift <= {1'b1, tx_byte, 1'b0};
            tx_cnt   <= 9'd0;
            tx_bits  <= 4'd0;
            tx_busy  <= 1'b1;
            tx       <= 1'b0;
        end else if (tx_busy) begin
            if (tx_cnt == 9'(BIT_CYC-1)) begin
                tx_cnt   <= 9'd0;
                tx_bits  <= tx_bits+4'd1;
                tx_shift <= {1'b1, tx_shift[9:1]};
                tx       <= tx_shift[1];
                if (tx_bits==4'd9) begin tx_busy<=1'b0; tx<=1'b1; end
            end else tx_cnt<=tx_cnt+9'd1;
        end
    end
end

// ════════════════════════════════════════════════════════════════════
// 3. TX Arbiter — Fix double-kick: dùng tx_busy_d để đảm bảo
//    arbiter không fire trong cycle ngay sau khi tx_send=1
// ════════════════════════════════════════════════════════════════════
reg [7:0] frame_byte; reg frame_req;
reg [7:0] debug_byte; reg debug_req;
reg       tx_busy_d;   // tx_busy delay 1 cycle

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) tx_busy_d<=1'b0;
    else        tx_busy_d<=tx_busy;
end

// Arbiter chỉ fire khi cả tx_busy lẫn tx_busy_d đều = 0
// (tức là TX thực sự rảnh ít nhất 2 cycle)
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin tx_byte<=8'd0; tx_send<=1'b0; end
    else begin
        tx_send <= 1'b0;
        if (!tx_busy && !tx_busy_d && !tx_send) begin
            if (frame_req) begin
                tx_byte <= frame_byte; tx_send <= 1'b1;
            end else if (debug_req) begin
                tx_byte <= debug_byte; tx_send <= 1'b1;
            end
        end
    end
end

// ════════════════════════════════════════════════════════════════════
// 4. Frame Parser + Response — 1 always block, no multiple drivers
// ════════════════════════════════════════════════════════════════════
localparam FP_IDLE=3'd0, FP_CMD=3'd1, FP_ADDR=3'd2,
           FP_LEN=3'd3,  FP_DATA=3'd4, FP_CHK=3'd5;

reg [2:0]  fp_state;
reg [7:0]  fp_cmd, fp_addr_r, fp_len;
reg [7:0]  fp_d0, fp_d1, fp_d2, fp_d3;
reg [7:0]  fp_chk;
reg [1:0]  fp_didx;

// Response buffer — SINGLE source của truth
reg [7:0]  resp [0:6];
reg [2:0]  resp_total;
reg [2:0]  resp_idx;
reg        resp_busy;
reg        build_read_resp; // pulse: yêu cầu build READ response
reg        build_read_resp_d; // delay 1 cycle — đợi reg_rdata valid

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        fp_state<=FP_IDLE; fp_chk<=8'd0; fp_didx<=2'd0;
        reg_wen<=1'b0; reg_ren<=1'b0; uart_kick<=1'b0;
        reg_addr<=8'd0; reg_wdata<=32'd0;
        resp_busy<=1'b0; resp_idx<=3'd0; resp_total<=3'd0;
        frame_req<=1'b0; frame_byte<=8'd0;
        fp_cmd<=8'd0; fp_addr_r<=8'd0; fp_len<=8'd0;
        fp_d0<=8'd0; fp_d1<=8'd0; fp_d2<=8'd0; fp_d3<=8'd0;
        resp[0]<=8'd0; resp[1]<=8'd0; resp[2]<=8'd0; resp[3]<=8'd0;
        resp[4]<=8'd0; resp[5]<=8'd0; resp[6]<=8'd0;
        build_read_resp<=1'b0;
        build_read_resp_d<=1'b0;
    end else begin
        reg_wen           <= 1'b0;
        reg_ren           <= 1'b0;
        uart_kick         <= 1'b0;
        frame_req         <= 1'b0;
        build_read_resp   <= 1'b0;
        // Fix: delay 1 cycle để reg_rdata kịp cập nhật
        build_read_resp_d <= build_read_resp;

        // ── RX Parser ────────────────────────────────────────────
        if (rx_valid) begin
            case (fp_state)
                FP_IDLE: if (rx_data==8'h55) fp_state<=FP_CMD;

                FP_CMD: begin
                    fp_cmd<=rx_data; fp_chk<=rx_data; fp_state<=FP_ADDR;
                end

                FP_ADDR: begin
                    fp_addr_r<=rx_data; fp_chk<=fp_chk^rx_data; fp_state<=FP_LEN;
                end

                FP_LEN: begin
                    fp_len<=rx_data; fp_chk<=fp_chk^rx_data; fp_didx<=2'd0;
                    fp_state<=(rx_data==8'h0) ? FP_CHK : FP_DATA;
                end

                FP_DATA: begin
                    fp_chk<=fp_chk^rx_data;
                    case (fp_didx)
                        2'd0: fp_d0<=rx_data;
                        2'd1: fp_d1<=rx_data;
                        2'd2: fp_d2<=rx_data;
                        2'd3: fp_d3<=rx_data;
                    endcase
                    if (fp_didx==fp_len[1:0]-2'd1) fp_state<=FP_CHK;
                    else fp_didx<=fp_didx+2'd1;
                end

                FP_CHK: begin
                    fp_state<=FP_IDLE;
                    if (rx_data==fp_chk) begin
                        case (fp_cmd)
                            8'h01: begin // WRITE_REG
                                reg_addr  <=fp_addr_r; reg_wen<=1'b1;
                                reg_wdata <={fp_d3,fp_d2,fp_d1,fp_d0};
                                resp[0]<=8'h55; resp[1]<=8'hAA; resp[2]<=8'hAA;
                                resp_total<=3'd3; resp_idx<=3'd0; resp_busy<=1'b1;
                            end
                            8'h02: begin // READ_REG
                                reg_addr<=fp_addr_r; reg_ren<=1'b1;
                                build_read_resp<=1'b1;
                            end
                            8'h03: begin // KICK
                                uart_kick<=1'b1;
                                resp[0]<=8'h55; resp[1]<=8'hAA; resp[2]<=8'hAA;
                                resp_total<=3'd3; resp_idx<=3'd0; resp_busy<=1'b1;
                            end
                            8'h04: begin // GET_STATUS
                                reg_addr<=8'h10; reg_ren<=1'b1;
                                build_read_resp<=1'b1;
                            end
                            default: begin
                                resp[0]<=8'h55; resp[1]<=8'hFF; resp[2]<=8'hFF;
                                resp_total<=3'd3; resp_idx<=3'd0; resp_busy<=1'b1;
                            end
                        endcase
                    end else begin
                        resp[0]<=8'h55; resp[1]<=8'hFF; resp[2]<=8'hFF;
                        resp_total<=3'd3; resp_idx<=3'd0; resp_busy<=1'b1;
                    end
                end
                default: fp_state<=FP_IDLE;
            endcase
        end

        // ── Build READ response (2 cycle sau reg_ren) ────────────
        // Dùng _d để đợi reg_rdata valid
        if (build_read_resp_d) begin
            resp[0] <= 8'h55;
            resp[1] <= 8'hAA;
            resp[2] <= reg_rdata[7:0];
            resp[3] <= reg_rdata[15:8];
            resp[4] <= reg_rdata[23:16];
            resp[5] <= reg_rdata[31:24];
            resp[6] <= 8'hAA ^ reg_rdata[7:0] ^ reg_rdata[15:8]
                             ^ reg_rdata[23:16] ^ reg_rdata[31:24];
            resp_total<=3'd7; resp_idx<=3'd0; resp_busy<=1'b1;
        end

        // ── Send response bytes ───────────────────────────────────
        if (resp_busy && !tx_busy && !frame_req) begin
            case (resp_idx)
                3'd0: frame_byte<=resp[0];
                3'd1: frame_byte<=resp[1];
                3'd2: frame_byte<=resp[2];
                3'd3: frame_byte<=resp[3];
                3'd4: frame_byte<=resp[4];
                3'd5: frame_byte<=resp[5];
                3'd6: frame_byte<=resp[6];
                default: frame_byte<=8'd0;
            endcase
            frame_req<=1'b1;
            if (resp_idx==resp_total-3'd1) resp_busy<=1'b0;
            else resp_idx<=resp_idx+3'd1;
        end
    end
end

// ════════════════════════════════════════════════════════════════════
// 5. Debug Log — Flat 1D ROM (tránh lỗi 2D array trên Gowin)
// Layout: msg_id * 24 + byte_idx
// Msg 0: "=== WDG READY ===\r\n" (19 bytes) @ addr 0
// Msg 1: "[WDG] DISABLED\r\n"   (16 bytes) @ addr 24
// Msg 2: "[WDG] ARMING...\r\n"  (17 bytes) @ addr 48
// Msg 3: "[WDG] RUNNING\r\n"    (15 bytes) @ addr 72
// Msg 4: "[WDG] KICK!\r\n"      (13 bytes) @ addr 96
// Msg 5: "[WDG] TIMEOUT!\r\n"   (16 bytes) @ addr 120
// Msg 6: "[WDG] RECOVER\r\n"    (15 bytes) @ addr 144
// ════════════════════════════════════════════════════════════════════
reg [7:0] msg_rom [0:167];  // 7 msg x 24 bytes = 168
reg [4:0] msg_len_rom [0:6];
reg [7:0] rom_addr;         // = dbg_msg*24 + dbg_idx

initial begin
    // Msg 0: "=== WDG READY ===\r\n"
    msg_rom[0]="="; msg_rom[1]="="; msg_rom[2]="="; msg_rom[3]=" ";
    msg_rom[4]="W"; msg_rom[5]="D"; msg_rom[6]="G"; msg_rom[7]=" ";
    msg_rom[8]="R"; msg_rom[9]="E"; msg_rom[10]="A"; msg_rom[11]="D";
    msg_rom[12]="Y"; msg_rom[13]=" "; msg_rom[14]="="; msg_rom[15]="=";
    msg_rom[16]="="; msg_rom[17]=8'h0D; msg_rom[18]=8'h0A;
    msg_len_rom[0]=5'd19;

    // Msg 1: "[WDG] DISABLED\r\n" @ 24
    msg_rom[24]="["; msg_rom[25]="W"; msg_rom[26]="D"; msg_rom[27]="G";
    msg_rom[28]="]"; msg_rom[29]=" "; msg_rom[30]="D"; msg_rom[31]="I";
    msg_rom[32]="S"; msg_rom[33]="A"; msg_rom[34]="B"; msg_rom[35]="L";
    msg_rom[36]="E"; msg_rom[37]="D"; msg_rom[38]=8'h0D; msg_rom[39]=8'h0A;
    msg_len_rom[1]=5'd16;

    // Msg 2: "[WDG] ARMING...\r\n" @ 48
    msg_rom[48]="["; msg_rom[49]="W"; msg_rom[50]="D"; msg_rom[51]="G";
    msg_rom[52]="]"; msg_rom[53]=" "; msg_rom[54]="A"; msg_rom[55]="R";
    msg_rom[56]="M"; msg_rom[57]="I"; msg_rom[58]="N"; msg_rom[59]="G";
    msg_rom[60]="."; msg_rom[61]="."; msg_rom[62]="."; msg_rom[63]=8'h0D;
    msg_rom[64]=8'h0A;
    msg_len_rom[2]=5'd17;

    // Msg 3: "[WDG] RUNNING\r\n" @ 72
    msg_rom[72]="["; msg_rom[73]="W"; msg_rom[74]="D"; msg_rom[75]="G";
    msg_rom[76]="]"; msg_rom[77]=" "; msg_rom[78]="R"; msg_rom[79]="U";
    msg_rom[80]="N"; msg_rom[81]="N"; msg_rom[82]="I"; msg_rom[83]="N";
    msg_rom[84]="G"; msg_rom[85]=8'h0D; msg_rom[86]=8'h0A;
    msg_len_rom[3]=5'd15;

    // Msg 4: "[WDG] KICK!\r\n" @ 96
    msg_rom[96]="["; msg_rom[97]="W"; msg_rom[98]="D"; msg_rom[99]="G";
    msg_rom[100]="]"; msg_rom[101]=" "; msg_rom[102]="K"; msg_rom[103]="I";
    msg_rom[104]="C"; msg_rom[105]="K"; msg_rom[106]="!"; msg_rom[107]=8'h0D;
    msg_rom[108]=8'h0A;
    msg_len_rom[4]=5'd13;

    // Msg 5: "[WDG] TIMEOUT!\r\n" @ 120
    msg_rom[120]="["; msg_rom[121]="W"; msg_rom[122]="D"; msg_rom[123]="G";
    msg_rom[124]="]"; msg_rom[125]=" "; msg_rom[126]="T"; msg_rom[127]="I";
    msg_rom[128]="M"; msg_rom[129]="E"; msg_rom[130]="O"; msg_rom[131]="U";
    msg_rom[132]="T"; msg_rom[133]="!"; msg_rom[134]=8'h0D; msg_rom[135]=8'h0A;
    msg_len_rom[5]=5'd16;

    // Msg 6: "[WDG] RECOVER\r\n" @ 144
    msg_rom[144]="["; msg_rom[145]="W"; msg_rom[146]="D"; msg_rom[147]="G";
    msg_rom[148]="]"; msg_rom[149]=" "; msg_rom[150]="R"; msg_rom[151]="E";
    msg_rom[152]="C"; msg_rom[153]="O"; msg_rom[154]="V"; msg_rom[155]="E";
    msg_rom[156]="R"; msg_rom[157]=8'h0D; msg_rom[158]=8'h0A;
    msg_len_rom[6]=5'd15;
end

// ROM address = msg_id * 24 + byte_idx
// Tính: addr = {dbg_msg, 3'b0} + {dbg_msg, 1'b0} + {dbg_msg} + dbg_idx
//      = dbg_msg*8 + dbg_msg*16 = dbg_msg*24 + dbg_idx
// Dùng wire để tránh 1-cycle delay
wire [7:0] rom_addr = {1'b0, dbg_msg, 4'b0} + {2'b0, dbg_msg, 3'b0} + {5'b0, dbg_idx};
// = dbg_msg*16 + dbg_msg*8 + dbg_idx = dbg_msg*24 + dbg_idx

reg en_prev_d, fault_prev_d, enout_prev_d, kick_prev_d;
reg boot_sent;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        en_prev_d<=1'b0; fault_prev_d<=1'b0;
        enout_prev_d<=1'b0; kick_prev_d<=1'b0;
    end else begin
        en_prev_d    <= en_effective;
        fault_prev_d <= fault_active;
        enout_prev_d <= enout;
        kick_prev_d  <= wdi_kick;
    end
end

// 4-state debug sender — tránh overwrite byte
localparam DBG_IDLE=2'd0, DBG_LOAD=2'd1,
           DBG_WAIT_START=2'd2, DBG_WAIT_DONE=2'd3;
reg [1:0]  dbg_state;
reg [2:0]  dbg_msg;
reg [4:0]  dbg_idx;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        dbg_state<=DBG_IDLE; dbg_msg<=3'd0;
        dbg_idx<=5'd0; debug_req<=1'b0; debug_byte<=8'd0;
        boot_sent<=1'b0;
    end else begin
        debug_req<=1'b0;
        case (dbg_state)
            DBG_IDLE: begin
                if (!boot_sent && !resp_busy && !tx_busy) begin
                    dbg_msg<=3'd0; dbg_idx<=5'd0;
                    dbg_state<=DBG_LOAD; boot_sent<=1'b1;
                end else if (!resp_busy && !tx_busy) begin
                    if (fault_active && !fault_prev_d)
                        begin dbg_msg<=3'd5; dbg_idx<=5'd0; dbg_state<=DBG_LOAD; end
                    else if (!fault_active && fault_prev_d && en_effective)
                        begin dbg_msg<=3'd6; dbg_idx<=5'd0; dbg_state<=DBG_LOAD; end
                    else if (wdi_kick && !kick_prev_d)
                        begin dbg_msg<=3'd4; dbg_idx<=5'd0; dbg_state<=DBG_LOAD; end
                    else if (enout && !enout_prev_d)
                        begin dbg_msg<=3'd3; dbg_idx<=5'd0; dbg_state<=DBG_LOAD; end
                    else if (en_effective && !en_prev_d && !enout)
                        begin dbg_msg<=3'd2; dbg_idx<=5'd0; dbg_state<=DBG_LOAD; end
                    else if (!en_effective && en_prev_d)
                        begin dbg_msg<=3'd1; dbg_idx<=5'd0; dbg_state<=DBG_LOAD; end
                end
            end

            DBG_LOAD: begin
                if (!tx_busy && !frame_req && !debug_req) begin
                    debug_byte <= msg_rom[rom_addr];
                    debug_req  <= 1'b1;
                    dbg_state  <= DBG_WAIT_START;
                end
            end

            DBG_WAIT_START: begin
                if (tx_busy) dbg_state<=DBG_WAIT_DONE;
            end

            DBG_WAIT_DONE: begin
                if (!tx_busy) begin
                    if (dbg_idx==msg_len_rom[dbg_msg]-5'd1)
                        dbg_state<=DBG_IDLE;
                    else begin
                        dbg_idx  <=dbg_idx+5'd1;
                        dbg_state<=DBG_LOAD;
                    end
                end
            end

            default: dbg_state<=DBG_IDLE;
        endcase
    end
end

endmodule
*/
// =============================================================================
// Module   : uart_frame
// Desc     : UART RX/TX 115200 8N1 + Frame Parser + Debug Log
// Fixed    : Combinational ROM for Gowin compatibility (Fixes BRAM Endianness/Latency)
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
localparam integer BIT_CYC  = (CLK_MHZ * 1000000) / BAUD;
localparam integer HALF_CYC = BIT_CYC / 2;

// ════════════════════════════════════════════════════════════════════
// 1. UART RX
// ════════════════════════════════════════════════════════════════════
reg rx_s1, rx_s2;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin rx_s1<=1'b1; rx_s2<=1'b1; end
    else        begin rx_s1<=rx; rx_s2<=rx_s1; end
end

localparam RX_IDLE=2'd0, RX_START=2'd1, RX_DATA=2'd2, RX_STOP=2'd3;
reg [1:0]   rx_state;
reg [8:0]   rx_cnt;     // 9 bit
reg [3:0]   rx_bit;     // 4 bit
reg [7:0]   rx_shift;
reg [7:0]   rx_data;
reg         rx_valid;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_state<=RX_IDLE; rx_cnt<=9'd0;
        rx_bit<=4'd0; rx_shift<=8'd0;
        rx_data<=8'd0; rx_valid<=1'b0;
    end else begin
        rx_valid <= 1'b0;
        case (rx_state)
            RX_IDLE: begin
                if (!rx_s2) begin
                    rx_state <= RX_START;
                    rx_cnt   <= 9'd0;
                end
            end
            RX_START: begin
                if (rx_cnt == 9'(HALF_CYC-1)) begin
                    rx_cnt <= 9'd0;
                    if (!rx_s2) begin rx_state<=RX_DATA; rx_bit<=4'd0; end
                    else         rx_state<=RX_IDLE;
                end else rx_cnt<=rx_cnt+9'd1;
            end
            RX_DATA: begin
                if (rx_cnt == 9'(BIT_CYC-1)) begin
                    rx_cnt   <= 9'd0;
                    rx_shift <= {rx_s2, rx_shift[7:1]};
                    if (rx_bit==4'd7) rx_state<=RX_STOP;
                    else rx_bit<=rx_bit+4'd1;
                end else rx_cnt<=rx_cnt+9'd1;
            end
            RX_STOP: begin
                if (rx_cnt == 9'(BIT_CYC-1)) begin
                    rx_cnt   <= 9'd0;
                    rx_state <= RX_IDLE;
                    if (rx_s2) begin rx_data<=rx_shift; rx_valid<=1'b1; end
                end else rx_cnt<=rx_cnt+9'd1;
            end
            default: rx_state<=RX_IDLE;
        endcase
    end
end

// ════════════════════════════════════════════════════════════════════
// 2. TX Engine
// ════════════════════════════════════════════════════════════════════
reg [7:0]  tx_byte;
reg        tx_send;
reg        tx_busy;
reg [8:0]  tx_cnt;
reg [9:0]  tx_shift;
reg [3:0]  tx_bits;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tx<=1'b1; tx_cnt<=9'd0; tx_bits<=4'd0;
        tx_shift<=10'h3FF; tx_busy<=1'b0;
    end else begin
        if (!tx_busy && tx_send) begin
            tx_shift <= {1'b1, tx_byte, 1'b0};
            tx_cnt   <= 9'd0;
            tx_bits  <= 4'd0;
            tx_busy  <= 1'b1;
            tx       <= 1'b0;
        end else if (tx_busy) begin
            if (tx_cnt == 9'(BIT_CYC-1)) begin
                tx_cnt   <= 9'd0;
                tx_bits  <= tx_bits+4'd1;
                tx_shift <= {1'b1, tx_shift[9:1]};
                tx       <= tx_shift[1];
                if (tx_bits==4'd9) begin tx_busy<=1'b0; tx<=1'b1; end
            end else tx_cnt<=tx_cnt+9'd1;
        end
    end
end

// ════════════════════════════════════════════════════════════════════
// 3. TX Arbiter
// ════════════════════════════════════════════════════════════════════
reg [7:0] frame_byte; reg frame_req;
reg [7:0] debug_byte; reg debug_req;
reg       tx_busy_d;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) tx_busy_d<=1'b0;
    else        tx_busy_d<=tx_busy;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin tx_byte<=8'd0; tx_send<=1'b0; end
    else begin
        tx_send <= 1'b0;
        if (!tx_busy && !tx_busy_d && !tx_send) begin
            if (frame_req) begin
                tx_byte <= frame_byte; tx_send <= 1'b1;
            end else if (debug_req) begin
                tx_byte <= debug_byte; tx_send <= 1'b1;
            end
        end
    end
end

// ════════════════════════════════════════════════════════════════════
// 4. Frame Parser + Response
// ════════════════════════════════════════════════════════════════════
localparam FP_IDLE=3'd0, FP_CMD=3'd1, FP_ADDR=3'd2,
           FP_LEN=3'd3,  FP_DATA=3'd4, FP_CHK=3'd5;

reg [2:0]  fp_state;
reg [7:0]  fp_cmd, fp_addr_r, fp_len;
reg [7:0]  fp_d0, fp_d1, fp_d2, fp_d3;
reg [7:0]  fp_chk;
reg [1:0]  fp_didx;

reg [7:0]  resp [0:6];
reg [2:0]  resp_total;
reg [2:0]  resp_idx;
reg        resp_busy;
reg        build_read_resp;
reg        build_read_resp_d;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        fp_state<=FP_IDLE; fp_chk<=8'd0; fp_didx<=2'd0;
        reg_wen<=1'b0; reg_ren<=1'b0; uart_kick<=1'b0;
        reg_addr<=8'd0; reg_wdata<=32'd0;
        resp_busy<=1'b0; resp_idx<=3'd0; resp_total<=3'd0;
        frame_req<=1'b0; frame_byte<=8'd0;
        fp_cmd<=8'd0; fp_addr_r<=8'd0; fp_len<=8'd0;
        fp_d0<=8'd0; fp_d1<=8'd0; fp_d2<=8'd0; fp_d3<=8'd0;
        resp[0]<=8'd0; resp[1]<=8'd0; resp[2]<=8'd0; resp[3]<=8'd0;
        resp[4]<=8'd0; resp[5]<=8'd0; resp[6]<=8'd0;
        build_read_resp<=1'b0;
        build_read_resp_d<=1'b0;
    end else begin
        reg_wen           <= 1'b0;
        reg_ren           <= 1'b0;
        uart_kick         <= 1'b0;
        frame_req         <= 1'b0;
        build_read_resp   <= 1'b0;
        build_read_resp_d <= build_read_resp;

        if (rx_valid) begin
            case (fp_state)
                FP_IDLE: if (rx_data==8'h55) fp_state<=FP_CMD;
                FP_CMD: begin
                    fp_cmd<=rx_data; fp_chk<=rx_data; fp_state<=FP_ADDR;
                end
                FP_ADDR: begin
                    fp_addr_r<=rx_data; fp_chk<=fp_chk^rx_data; fp_state<=FP_LEN;
                end
                FP_LEN: begin
                    fp_len<=rx_data; fp_chk<=fp_chk^rx_data; fp_didx<=2'd0;
                    fp_state<=(rx_data==8'h0) ? FP_CHK : FP_DATA;
                end
                FP_DATA: begin
                    fp_chk<=fp_chk^rx_data;
                    case (fp_didx)
                        2'd0: fp_d0<=rx_data;
                        2'd1: fp_d1<=rx_data;
                        2'd2: fp_d2<=rx_data;
                        2'd3: fp_d3<=rx_data;
                    endcase
                    if (fp_didx==fp_len[1:0]-2'd1) fp_state<=FP_CHK;
                    else fp_didx<=fp_didx+2'd1;
                end
                FP_CHK: begin
                    fp_state<=FP_IDLE;
                    if (rx_data==fp_chk) begin
                        case (fp_cmd)
                            8'h01: begin // WRITE
                                reg_addr  <=fp_addr_r; reg_wen<=1'b1;
                                reg_wdata <={fp_d3,fp_d2,fp_d1,fp_d0};
                                resp[0]<=8'h55; resp[1]<=8'hAA; resp[2]<=8'hAA;
                                resp_total<=3'd3; resp_idx<=3'd0; resp_busy<=1'b1;
                            end
                            8'h02: begin // READ
                                reg_addr<=fp_addr_r; reg_ren<=1'b1;
                                build_read_resp<=1'b1;
                            end
                            8'h03: begin // KICK
                                uart_kick<=1'b1;
                                resp[0]<=8'h55; resp[1]<=8'hAA; resp[2]<=8'hAA;
                                resp_total<=3'd3; resp_idx<=3'd0; resp_busy<=1'b1;
                            end
                            8'h04: begin // GET_STATUS
                                reg_addr<=8'h10; reg_ren<=1'b1;
                                build_read_resp<=1'b1;
                            end
                            default: begin
                                resp[0]<=8'h55; resp[1]<=8'hFF; resp[2]<=8'hFF;
                                resp_total<=3'd3; resp_idx<=3'd0; resp_busy<=1'b1;
                            end
                        endcase
                    end else begin
                        resp[0]<=8'h55; resp[1]<=8'hFF; resp[2]<=8'hFF;
                        resp_total<=3'd3; resp_idx<=3'd0; resp_busy<=1'b1;
                    end
                end
                default: fp_state<=FP_IDLE;
            endcase
        end

        if (build_read_resp_d) begin
            resp[0] <= 8'h55;
            resp[1] <= 8'hAA;
            resp[2] <= reg_rdata[7:0];
            resp[3] <= reg_rdata[15:8];
            resp[4] <= reg_rdata[23:16];
            resp[5] <= reg_rdata[31:24];
            resp[6] <= 8'hAA ^ reg_rdata[7:0] ^ reg_rdata[15:8]
                             ^ reg_rdata[23:16] ^ reg_rdata[31:24];
            resp_total<=3'd7; resp_idx<=3'd0; resp_busy<=1'b1;
        end

        // ── Send response bytes ───────────────────────────────────
        // FIX: Đợi tín hiệu !tx_busy_d và !tx_send để chống trôi byte
        if (resp_busy && !tx_busy && !tx_busy_d && !tx_send && !frame_req) begin
            case (resp_idx)
                3'd0: frame_byte<=resp[0];
                3'd1: frame_byte<=resp[1];
                3'd2: frame_byte<=resp[2];
                3'd3: frame_byte<=resp[3];
                3'd4: frame_byte<=resp[4];
                3'd5: frame_byte<=resp[5];
                3'd6: frame_byte<=resp[6];
                default: frame_byte<=8'd0;
            endcase
            frame_req<=1'b1;
            if (resp_idx==resp_total-3'd1) resp_busy<=1'b0;
            else resp_idx<=resp_idx+3'd1;
        end
    end
end

// ════════════════════════════════════════════════════════════════════
// 5. Debug Log — Combinational ROM (Fixes Gowin BRAM packing & latency)
// ════════════════════════════════════════════════════════════════════
localparam DBG_IDLE=2'd0, DBG_LOAD=2'd1,
           DBG_WAIT_START=2'd2, DBG_WAIT_DONE=2'd3;

reg [1:0]  dbg_state;
reg [2:0]  dbg_msg;
reg [4:0]  dbg_idx;

reg [4:0] msg_len;
always @(*) begin
    case (dbg_msg)
        3'd0: msg_len = 5'd19;
        3'd1: msg_len = 5'd16;
        3'd2: msg_len = 5'd17;
        3'd3: msg_len = 5'd15;
        3'd4: msg_len = 5'd13;
        3'd5: msg_len = 5'd16;
        3'd6: msg_len = 5'd15;
        default: msg_len = 5'd0;
    endcase
end

reg [7:0] rom_char;
always @(*) begin
    rom_char = 8'h20; // Default: Space
    case (dbg_msg)
        3'd0: case(dbg_idx) // "=== WDG READY ===\r\n"
            5'd0:rom_char="="; 5'd1:rom_char="="; 5'd2:rom_char="="; 5'd3:rom_char=" ";
            5'd4:rom_char="W"; 5'd5:rom_char="D"; 5'd6:rom_char="G"; 5'd7:rom_char=" ";
            5'd8:rom_char="R"; 5'd9:rom_char="E"; 5'd10:rom_char="A"; 5'd11:rom_char="D";
            5'd12:rom_char="Y"; 5'd13:rom_char=" "; 5'd14:rom_char="="; 5'd15:rom_char="=";
            5'd16:rom_char="="; 5'd17:rom_char=8'h0D; 5'd18:rom_char=8'h0A;
            default: rom_char=8'h00;
        endcase
        3'd1: case(dbg_idx) // "[WDG] DISABLED\r\n"
            5'd0:rom_char="["; 5'd1:rom_char="W"; 5'd2:rom_char="D"; 5'd3:rom_char="G";
            5'd4:rom_char="]"; 5'd5:rom_char=" "; 5'd6:rom_char="D"; 5'd7:rom_char="I";
            5'd8:rom_char="S"; 5'd9:rom_char="A"; 5'd10:rom_char="B"; 5'd11:rom_char="L";
            5'd12:rom_char="E"; 5'd13:rom_char="D"; 5'd14:rom_char=8'h0D; 5'd15:rom_char=8'h0A;
            default: rom_char=8'h00;
        endcase
        3'd2: case(dbg_idx) // "[WDG] ARMING...\r\n"
            5'd0:rom_char="["; 5'd1:rom_char="W"; 5'd2:rom_char="D"; 5'd3:rom_char="G";
            5'd4:rom_char="]"; 5'd5:rom_char=" "; 5'd6:rom_char="A"; 5'd7:rom_char="R";
            5'd8:rom_char="M"; 5'd9:rom_char="I"; 5'd10:rom_char="N"; 5'd11:rom_char="G";
            5'd12:rom_char="."; 5'd13:rom_char="."; 5'd14:rom_char="."; 5'd15:rom_char=8'h0D;
            5'd16:rom_char=8'h0A; default: rom_char=8'h00;
        endcase
        3'd3: case(dbg_idx) // "[WDG] RUNNING\r\n"
            5'd0:rom_char="["; 5'd1:rom_char="W"; 5'd2:rom_char="D"; 5'd3:rom_char="G";
            5'd4:rom_char="]"; 5'd5:rom_char=" "; 5'd6:rom_char="R"; 5'd7:rom_char="U";
            5'd8:rom_char="N"; 5'd9:rom_char="N"; 5'd10:rom_char="I"; 5'd11:rom_char="N";
            5'd12:rom_char="G"; 5'd13:rom_char=8'h0D; 5'd14:rom_char=8'h0A;
            default: rom_char=8'h00;
        endcase
        3'd4: case(dbg_idx) // "[WDG] KICK!\r\n"
            5'd0:rom_char="["; 5'd1:rom_char="W"; 5'd2:rom_char="D"; 5'd3:rom_char="G";
            5'd4:rom_char="]"; 5'd5:rom_char=" "; 5'd6:rom_char="K"; 5'd7:rom_char="I";
            5'd8:rom_char="C"; 5'd9:rom_char="K"; 5'd10:rom_char="!"; 5'd11:rom_char=8'h0D;
            5'd12:rom_char=8'h0A; default: rom_char=8'h00;
        endcase
        3'd5: case(dbg_idx) // "[WDG] TIMEOUT!\r\n"
            5'd0:rom_char="["; 5'd1:rom_char="W"; 5'd2:rom_char="D"; 5'd3:rom_char="G";
            5'd4:rom_char="]"; 5'd5:rom_char=" "; 5'd6:rom_char="T"; 5'd7:rom_char="I";
            5'd8:rom_char="M"; 5'd9:rom_char="E"; 5'd10:rom_char="O"; 5'd11:rom_char="U";
            5'd12:rom_char="T"; 5'd13:rom_char="!"; 5'd14:rom_char=8'h0D; 5'd15:rom_char=8'h0A;
            default: rom_char=8'h00;
        endcase
        3'd6: case(dbg_idx) // "[WDG] RECOVER\r\n"
            5'd0:rom_char="["; 5'd1:rom_char="W"; 5'd2:rom_char="D"; 5'd3:rom_char="G";
            5'd4:rom_char="]"; 5'd5:rom_char=" "; 5'd6:rom_char="R"; 5'd7:rom_char="E";
            5'd8:rom_char="C"; 5'd9:rom_char="O"; 5'd10:rom_char="V"; 5'd11:rom_char="E";
            5'd12:rom_char="R"; 5'd13:rom_char=8'h0D; 5'd14:rom_char=8'h0A;
            default: rom_char=8'h00;
        endcase
        default: rom_char = 8'h00;
    endcase
end

reg en_prev_d, fault_prev_d, enout_prev_d, kick_prev_d;
reg boot_sent;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        en_prev_d<=1'b0; fault_prev_d<=1'b0;
        enout_prev_d<=1'b0; kick_prev_d<=1'b0;
    end else begin
        en_prev_d    <= en_effective;
        fault_prev_d <= fault_active;
        enout_prev_d <= enout;
        kick_prev_d  <= wdi_kick;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        dbg_state<=DBG_IDLE; dbg_msg<=3'd0;
        dbg_idx<=5'd0; debug_req<=1'b0; debug_byte<=8'd0;
        boot_sent<=1'b0;
    end else begin
        debug_req<=1'b0;
        case (dbg_state)
            DBG_IDLE: begin
                if (!boot_sent && !resp_busy && !tx_busy) begin
                    dbg_msg<=3'd0; dbg_idx<=5'd0;
                    dbg_state<=DBG_LOAD; boot_sent<=1'b1;
                end else if (!resp_busy && !tx_busy) begin
                    if (fault_active && !fault_prev_d)
                        begin dbg_msg<=3'd5; dbg_idx<=5'd0; dbg_state<=DBG_LOAD; end
                    else if (!fault_active && fault_prev_d && en_effective)
                        begin dbg_msg<=3'd6; dbg_idx<=5'd0; dbg_state<=DBG_LOAD; end
                    else if (wdi_kick && !kick_prev_d)
                        begin dbg_msg<=3'd4; dbg_idx<=5'd0; dbg_state<=DBG_LOAD; end
                    else if (enout && !enout_prev_d)
                        begin dbg_msg<=3'd3; dbg_idx<=5'd0; dbg_state<=DBG_LOAD; end
                    else if (en_effective && !en_prev_d && !enout)
                        begin dbg_msg<=3'd2; dbg_idx<=5'd0; dbg_state<=DBG_LOAD; end
                    else if (!en_effective && en_prev_d)
                        begin dbg_msg<=3'd1; dbg_idx<=5'd0; dbg_state<=DBG_LOAD; end
                end
            end

            DBG_LOAD: begin
                if (!tx_busy && !frame_req && !debug_req) begin
                    debug_byte <= rom_char;
                    debug_req  <= 1'b1;
                    dbg_state  <= DBG_WAIT_START;
                end
            end

            DBG_WAIT_START: begin
                if (tx_busy) dbg_state<=DBG_WAIT_DONE;
            end

            DBG_WAIT_DONE: begin
                if (!tx_busy) begin
                    if (dbg_idx == msg_len - 5'd1)
                        dbg_state<=DBG_IDLE;
                    else begin
                        dbg_idx  <=dbg_idx+5'd1;
                        dbg_state<=DBG_LOAD;
                    end
                end
            end

            default: dbg_state<=DBG_IDLE;
        endcase
    end
end

endmodule