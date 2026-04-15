# Watchdog Monitor (TPS3431-like) + UART Configuration

**Platform:** Kiwi 1P5 (Gowin GW1N-UV1P5) | **Clock:** 27 MHz  
**Contest:** FPGA Extended Contest 2026 — Preliminary Round RTL Design Test  
**Author:** Dong Truong Long

---

## Mô tả dự án

Thiết kế RTL mô phỏng chức năng watchdog supervisor của IC **TPS3431** (Texas Instruments), cho phép cấu hình tham số runtime qua UART, chạy trên board **Kiwi 1P5**.

### Tính năng chính

- Giám sát tín hiệu kick (WDI) từ nút nhấn S1 hoặc lệnh UART
- Phát hiện timeout và xuất tín hiệu lỗi WDO (active-low)
- Hỗ trợ Enable/Disable watchdog qua nút S2 hoặc UART
- Tín hiệu ENOUT báo hiệu hệ thống đã sẵn sàng sau arm_delay
- Cấu hình runtime: tWD, tRST, arm_delay qua UART 115200 bps
- Debug log tự động in trạng thái ra Serial Monitor

---

## Cấu trúc thư mục

```
watchdog_kiwi/
├── src/
│   ├── watchdog_top.sv     ← Top-level, power-on reset, wiring
│   ├── watchdog_core.sv    ← FSM chính + timers + debounce
│   ├── uart_frame.sv       ← UART RX/TX + frame parser + debug log
│   └── regfile.sv          ← Configuration registers + STATUS
├── tb/
│   └── tb_watchdog.sv      ← Testbench 5 test cases
├── constraints/
│   ├── kiwi1p5.cst         ← Pin assignment + IO standard
│   └── kiwi1p5.sdc         ← Timing constraint (27 MHz clock)
└── test_uart.py            ← Python script test UART từ PC
```

---

## Kiến trúc RTL

```
                    ┌─────────────────────────────────────────┐
  S1 (btn_wdi_n) ──►│                                         │
  S2 (btn_en_n)  ──►│  watchdog_core                          ├──► LED D3 (WDO)
                    │  - 2-FF Sync + Debounce 10ms            │
                    │  - FSM: DISABLED→ARMING→RUNNING→FAULT   ├──► LED D4 (ENOUT)
                    │  - Timers: tWD, tRST, arm_delay         │
                    └──────────┬──────────────────────────────┘
                               │ status
                    ┌──────────▼──────────────────────────────┐
                    │  regfile                                  │
                    │  CTRL / tWD_ms / tRST_ms /               │
                    │  arm_delay_us / STATUS                   │
                    └──────────┬──────────────────────────────┘
                               │ reg R/W
                    ┌──────────▼──────────────────────────────┐
  uart_rx ─────────►│  uart_frame                             ├──► uart_tx
                    │  - UART RX/TX 115200 8N1                │
                    │  - Frame parser + checksum XOR           │
                    │  - KICK command                          │
                    │  - Debug log → Serial Monitor            │
                    └─────────────────────────────────────────┘
```

---

## FSM Watchdog Core

```
         EN=1
DISABLED ──────► ARMING ──────────► RUNNING ──────────► FAULT
   ▲               │   arm_delay        │   tWD expired    │
   │               │   done             │                  │
   └───────────────┴────────────────────┴──────────────────┘
         EN=0 (từ bất kỳ state nào)         tRST done /
                                            CLR_FAULT
```

| State    | ENOUT | WDO     | Mô tả                             |
| -------- | ----- | ------- | --------------------------------- |
| DISABLED | 0     | 1 (ok)  | Watchdog tắt                      |
| ARMING   | 0     | 1 (ok)  | Đang đếm arm_delay, WDI bị ignore |
| RUNNING  | 1     | 1 (ok)  | Đang hoạt động bình thường        |
| FAULT    | 1     | 0 (lỗi) | Timeout! WDO kéo thấp trong tRST  |

---

## Tham số mặc định

| Tham số      | Giá trị | Mô tả                            |
| ------------ | ------- | -------------------------------- |
| tWD_ms       | 1600 ms | Watchdog timeout (CWD=NC mode)   |
| tRST_ms      | 200 ms  | WDO hold time khi fault          |
| arm_delay_us | 150 µs  | WDI ignore window sau khi enable |
| UART baud    | 115200  | 8N1, không parity                |

---

## Pin Assignment (Kiwi 1P5)

| Chức năng     | Board            | FPGA Pin | IO Standard       |
| ------------- | ---------------- | -------- | ----------------- |
| Clock 27 MHz  | Oscillator       | 4        | LVCMOS33          |
| WDI Kick      | Button S1 / KEY1 | 35       | LVCMOS33, PULL UP |
| Enable        | Button S2 / KEY2 | 36       | LVCMOS33, PULL UP |
| WDO (Fault)   | LED D3 / LED1    | 27       | LVCMOS33, 8mA     |
| ENOUT (Armed) | LED D4 / LED2    | 28       | LVCMOS33, 8mA     |
| UART RX       | USB-UART GWU2U   | 33       | LVCMOS33, PULL UP |
| UART TX       | USB-UART GWU2U   | 34       | LVCMOS33, 8mA     |

> **Lưu ý:** Không dùng pin 40 (RECONFIG_N) và pin 41 (JTAGSEL_N) làm GPIO.

---

## Open-Drain Emulation

Chọn **Approach B**: Push-pull outputs với active-low convention.

- `WDO = 0` → Fault (LED D3 sáng)
- `WDO = 1` → Normal (LED D3 tắt)
- `ENOUT = 1` → Watchdog armed (LED D4 sáng)

Lý do: GW1N-UV1P5 không hỗ trợ tri-state GPIO dễ dàng, push-pull đơn giản hơn và LED demo hoạt động đúng.

---

## UART Protocol

### Frame Format

```
[0x55] [CMD] [ADDR] [LEN] [DATA...] [CHK]
  ↑      ↑     ↑      ↑      ↑        ↑
Start  Lệnh  Địa chỉ Độ dài Data  XOR checksum

CHK = CMD ^ ADDR ^ LEN ^ DATA[0] ^ ... ^ DATA[N-1]
```

### Lệnh hỗ trợ

| CMD        | Hex  | Mô tả                                           |
| ---------- | ---- | ----------------------------------------------- |
| WRITE_REG  | 0x01 | Ghi register                                    |
| READ_REG   | 0x02 | Đọc register                                    |
| KICK       | 0x03 | Tạo 1 kick event (tương đương WDI falling edge) |
| GET_STATUS | 0x04 | Đọc nhanh STATUS register                       |

### Register Map

| Addr | Tên          | R/W | Mô tả                                                        |
| ---- | ------------ | --- | ------------------------------------------------------------ |
| 0x00 | CTRL         | R/W | bit0=EN_SW, bit1=WDI_SRC, bit2=CLR_FAULT                     |
| 0x04 | tWD_ms       | R/W | Watchdog timeout (ms), default 1600                          |
| 0x08 | tRST_ms      | R/W | WDO hold time (ms), default 200                              |
| 0x0C | arm_delay_us | R/W | WDI ignore window (µs), default 150                          |
| 0x10 | STATUS       | R   | bit0=EN_EFF, bit1=FAULT, bit2=ENOUT, bit3=WDO, bit4=KICK_SRC |

### Ví dụ frame

```
# Đọc STATUS
TX: 55 04 10 00 14
RX: 55 AA [D0 D1 D2 D3] [CHK]

# Ghi tWD = 5000ms (0x00001388)
TX: 55 01 04 04 88 13 00 00 CHK
CHK = 0x01^0x04^0x04^0x88^0x13^0x00^0x00 = 0x8E

# KICK qua UART
TX: 55 03 00 00 03
RX: 55 AA AA
```

---

## Hướng dẫn build (Gowin EDA)

### Bước 1 — Tạo project

```
File → New Project
Device: GW1N-UV1P5
```

### Bước 2 — Add files

```
Project → Add Files → chọn tất cả trong src/:
  ✅ watchdog_top.sv   ← set làm Top Module
  ✅ watchdog_core.sv
  ✅ uart_frame.sv
  ✅ regfile.sv
  ✅ kiwi1p5.cst
  ✅ kiwi1p5.sdc
  ❌ tb_watchdog.sv    ← KHÔNG add (chỉ dùng sim)
```

### Bước 3 — Synthesis → P&R → Bitstream

```
Process → Synthesize
Process → Place & Route
Process → Generate Bitstream
```

### Bước 4 — Nạp xuống board

```
Tools → Programmer
Cable: GWU2U (auto detect)
Frequency: 2.5 MHz
Operation: SRAM Program (test) / Flash (lưu vĩnh viễn)
File: impl/pnr/watchdog_top.fs
```

---

## Hướng dẫn test trên board

### Test thủ công (LED + Button)

| Hành động            | Serial Monitor      | LED D4 | LED D3 |
| -------------------- | ------------------- | ------ | ------ |
| Board khởi động      | `=== WDG READY ===` | TẮT    | TẮT    |
|                      | `[WDG] DISABLED`    | TẮT    | TẮT    |
| Nhấn S2              | `[WDG] ARMING...`   | TẮT    | TẮT    |
| (~150µs sau)         | `[WDG] RUNNING`     | SÁNG   | TẮT    |
| Nhấn S1              | `[WDG] KICK!`       | SÁNG   | TẮT    |
| Không nhấn S1 (1.6s) | `[WDG] TIMEOUT!`    | SÁNG   | SÁNG   |
| (200ms sau)          | `[WDG] RECOVER`     | SÁNG   | TẮT    |
| Thả S2               | `[WDG] DISABLED`    | TẮT    | TẮT    |

### Test UART bằng Python

```bash
# Cài thư viện
pip install pyserial

# Đổi PORT trong file
# Windows: PORT = 'COM3'  (xem Device Manager)
# Linux:   PORT = '/dev/ttyUSB0'

python test_uart.py
```

### Mở Serial Monitor

```
Baud rate : 115200
Data bits : 8
Parity    : None
Stop bits : 1
Tool      : PuTTY / Arduino IDE Serial Monitor / VS Code Serial Monitor
```

---

## Testbench (5 test cases)

```bash
# Chạy simulation với iverilog
iverilog -g2012 -o sim/tb src/watchdog_core.sv tb/tb_watchdog.sv
vvp sim/tb
gtkwave tb_watchdog.vcd
```

# Chạy simulation với Vivado

| TC  | Tên          | Mô tả                                 |
| --- | ------------ | ------------------------------------- |
| TC1 | Normal Kick  | Kick đúng hạn → không timeout         |
| TC2 | Timeout      | Không kick → WDO assert → tự recover  |
| TC3 | Disable      | EN=0, WDI bị ignore, ENOUT=0          |
| TC4 | arm_delay    | Kick trong arm window bị bỏ qua       |
| TC5 | Param change | Đổi tWD qua regfile, verify CLR_FAULT |

---

## Scoring

| Hạng mục              | Trọng số |
| --------------------- | -------- |
| Watchdog FSM + timing | 50%      |
| UART + register map   | 25%      |
| Testbench             | 15%      |
| Code style + README   | 10%      |
