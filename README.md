# Smart Main Control 2 Plus

Bo mạch **trung tâm điều khiển** tích hợp **STM32F407** và **ESP32**, dùng cho các ứng dụng robot, điều khiển động cơ DC 12V, giao tiếp ngoại vi và IoT. Thiết kế kế thừa ý tưởng từ board phát triển, nhưng tối ưu tích hợp phần cứng cho hệ thống điều khiển thực tế.

---

## 1. Chức năng chính
- **MCU chính:** STM32F407VGT6, xung nhịp 168 MHz, hỗ trợ nhiều timer, PWM, ADC, giao tiếp ngoại vi.
- **MCU phụ:** ESP32-S (Wi-Fi/Bluetooth, giao tiếp UART với STM32).
- **Nguồn:** đầu vào DC 12–24 V, hạ áp XL4015 (buck) → 5 V, sau đó qua LM1117-3.3 để cấp cho MCU/ESP32.
- **Điều khiển động cơ:** hỗ trợ 8 kênh PWM + DIR (điều khiển tối đa 8 động cơ DC thông qua mạch driver ngoài).
- **Cổng encoder:** 4 bộ encoder (TIM2, TIM3, TIM4, TIM5) để đo tốc độ/vị trí động cơ.
- **Giao tiếp ngoại vi:**
  - UART1–UART4 (header riêng).
  - SPI1 (MISO, MOSI, SCK, NSS).
  - I²C1 (SCL, SDA).
  - GPIO mở rộng qua header IO1–IO40 (nguồn 5 V, 3.3 V, GND kèm).
- **Màn hình LCD I²C** để hiển thị thông tin (SCL, SDA).
- **Chân cấu hình:** BOOT0, NRST, SWD (debug nạp code).
- **Đèn LED trạng thái, nút nhấn Reset/Program** cho ESP32.

---

## 2. Sơ đồ khối
```mermaid
flowchart TB
    VIN[DC 12–24 V] --> PWR[Power XL4015 → 5V, LM1117 → 3V3]
    PWR --> STM32[STM32F407VGT6]
    PWR --> ESP32[ESP32-S WiFi/BT]
    STM32 <-->|UART| ESP32
    STM32 -->|PWM+DIR| DRV[Motor Drivers (8 kênh DC)]
    ENC[Encoders x4] --> STM32
    LCD[LCD I2C] --> STM32
    IO[GPIO Expansion IO1–IO40] --> STM32
    STM32 -->|UART/SPI/I2C| EXT[External Modules]
```

---

## 3. Kết nối chính
### 3.1 Điều khiển động cơ
| Motor | PWM | DIR | Timer |
|-------|-----|-----|-------|
| M1    | PWM1 | DIR1 | TIM1 |
| M2    | PWM2 | DIR2 | TIM1 |
| M3    | PWM3 | DIR3 | TIM1 |
| M4    | PWM4 | DIR4 | TIM1 |
| M5    | PWM5 | DIR5 | TIM8 |
| M6    | PWM6 | DIR6 | TIM8 |
| M7    | PWM7 | DIR7 | TIM8 |
| M8    | PWM8 | DIR8 | TIM8 |

### 3.2 Encoder
- TIM2 CH1/CH2  
- TIM3 CH1/CH2  
- TIM4 CH1/CH2  
- TIM5 CH1/CH2

### 3.3 UART
- UART1: TX/RX header  
- UART2: TX/RX header (kết nối ESP32)  
- UART3: TX/RX header  
- UART4: TX/RX header

### 3.4 SPI
- MOSI, MISO, SCK, NSS ra header riêng

### 3.5 I/O mở rộng
- IO1–IO40 ra 2 header 2×14, cấp sẵn 3.3 V, 5 V, GND.  

---

## 4. Cấp nguồn
- **VIN:** DC 12–24 V qua jack DC.  
- **Buck XL4015:** tạo 5 V, dòng tối đa 3–5 A.  
- **LM1117-3.3:** tạo 3.3 V cho STM32/ESP32.  
- Có diode bảo vệ, cuộn cảm lọc, tụ bulk 220 µF/35 V và 330 µF/16 V.  

---

## 5. ESP32 Subsystem
- ESP32-S module, giao tiếp UART với STM32 (chọn qua jumper IO/UART).  
- Có nút **EN** và **IO0** để nạp firmware.  
- LED báo trạng thái, header nạp (TXD0, RXD0).  

---

## 6. LCD
- Kết nối I²C (SCL, SDA, +5 V, GND).  

---

## 7. Debug và nạp code
- **STM32:** SWD (SWDIO, SWCLK, NRST, +3.3, GND).  
- **ESP32:** UART0 + IO0, EN.  

---

## 8. Ứng dụng
- Robot di động điều khiển nhiều động cơ.  
- Bộ điều khiển trung tâm trong hệ thống tự động hóa.  
- Gateway IoT: ESP32 thu thập dữ liệu và gửi về server qua Wi-Fi.  
- Nghiên cứu/thi đấu Robocon.  

---

## 9. Thư mục khuyến nghị trong repo
```
SmartMainControl2Plus/
├── hardware/       # Schematic, PCB
├── firmware/
│   ├── stm32/      # CubeMX project
│   └── esp32/      # ESP-IDF project
├── docs/           # Datasheet, pinmap
└── examples/       # Demo motor, encoder, comms
```
