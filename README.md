# ESP32-S3 Flight Controller Balance Firmware

Firmware điều khiển cân bằng drone mini dùng **ESP32-S3 + GY-85 + SBUS + ESC OneShot125-like**.

Dự án này được xây dựng theo cấu trúc gần giống firmware flight controller đơn giản:

```text
app_main.c
  -> system.c
    -> stabilizer.c
      -> sensors.c
      -> estimator_complementary.c
      -> commander_sbus.c
      -> controller_pid.c
      -> power_distribution.c
      -> esc_oneshot_ledc.c
```

Mục tiêu hiện tại:

```text
- Đọc cảm biến GY-85: ADXL345 + ITG3205
- Ước tính roll/pitch bằng complementary filter
- Nhận điều khiển từ SBUS receiver
- Điều khiển attitude bằng Angle P + Rate PID
- Trộn lệnh ra 4 motor dạng X-Quad
- Xuất tín hiệu ESC bằng LEDC OneShot125-like
- Có arm/disarm/failsafe cơ bản
- Có WS2812 báo trạng thái
```

> ⚠️ Đây là firmware thử nghiệm cho drone DIY. Luôn tháo cánh khi test motor, sensor, mixer, PID hoặc failsafe.

---

## 1. Cấu hình phần cứng

### MCU

```text
ESP32-S3 Dev Module
ESP-IDF v5.3.x
```

### Cảm biến

```text
GY-85:
- ADXL345 accelerometer
- ITG3205 gyroscope
```

Kết nối I2C:

| Chức năng |  GPIO |
| --------- | ----: |
| I2C SDA   | GPIO8 |
| I2C SCL   | GPIO9 |

Trong code:

```c
#define I2C_SDA_GPIO        GPIO_NUM_8
#define I2C_SCL_GPIO        GPIO_NUM_9
#define I2C_FREQ_HZ         400000
```

Nếu motor làm nhiễu I2C, có thể giảm xuống:

```c
#define I2C_FREQ_HZ         100000
```

---

## 2. Mapping motor thực tế

Kết quả test thực tế bằng Arduino motor GPIO test:

```text
GPIO4 -> Motor 1
GPIO5 -> Motor 2
GPIO6 -> Motor 3
GPIO7 -> Motor 4
```

Vì vậy trong `main/board_config.h` dùng mapping:

```c
#define MOTOR1_GPIO         GPIO_NUM_4   // M1 = Rear-Left
#define MOTOR2_GPIO         GPIO_NUM_5   // M2 = Front-Left
#define MOTOR3_GPIO         GPIO_NUM_6   // M3 = Rear-Right
#define MOTOR4_GPIO         GPIO_NUM_7   // M4 = Front-Right
```

Layout drone nhìn từ trên xuống:

```text
                 FRONT
                  ↑

          M2 CCW          M4 CW
          GPIO5           GPIO7


          M1 CW           M3 CCW
          GPIO4           GPIO6

                  ↓
                 REAR
```

Bảng motor:

| Motor | Vị trí                   |  GPIO | Chiều quay |
| ----- | ------------------------ | ----: | ---------- |
| M1    | Rear-Left / Sau trái     | GPIO4 | CW         |
| M2    | Front-Left / Trước trái  | GPIO5 | CCW        |
| M3    | Rear-Right / Sau phải    | GPIO6 | CCW        |
| M4    | Front-Right / Trước phải | GPIO7 | CW         |

> Nếu motor quay sai chiều, đổi 2 dây bất kỳ giữa ESC và motor brushless đó.

---

## 3. SBUS receiver

SBUS dùng UART1 RX:

```c
#define SBUS_UART_NUM       UART_NUM_1
#define SBUS_RX_GPIO        GPIO_NUM_16
#define SBUS_RX_INVERT      1
```

Mapping kênh SBUS:

| SBUS channel | Chức năng  |
| ------------ | ---------- |
| CH1          | Roll       |
| CH2          | Pitch      |
| CH3          | Throttle   |
| CH4          | Yaw rate   |
| CH5          | Arm switch |

Trong code:

```c
#define SBUS_CH_ROLL        0
#define SBUS_CH_PITCH       1
#define SBUS_CH_THROTTLE    2
#define SBUS_CH_YAW         3
#define SBUS_CH_ARM         4
```

Giá trị SBUS chuẩn:

```c
#define SBUS_VALUE_MIN      172
#define SBUS_VALUE_MID      992
#define SBUS_VALUE_MAX      1811
```

Failsafe timeout:

```c
#define SBUS_FAILSAFE_TIMEOUT_MS  150
```

Nếu quá thời gian này không có frame SBUS mới, firmware đưa drone vào failsafe và motor về 0.

---

## 4. WS2812 status LED

Board ESP32-S3 Dev Module dùng WS2812 tại:

```c
#define STATUS_LED_GPIO     GPIO_NUM_48
#define STATUS_LED_NUM      1
```

Trạng thái LED:

| Trạng thái   | LED                   |
| ------------ | --------------------- |
| DISARMED     | Vàng nháy             |
| ARMING       | Xanh dương nháy nhanh |
| ARMED_IDLE   | Xanh lá               |
| FLYING       | Xanh dương            |
| FAILSAFE     | Đỏ nháy nhanh         |
| SENSOR_ERROR | Tím nháy nhanh        |

Cần thêm dependency:

```bash
idf.py add-dependency "espressif/led_strip"
```

---

## 5. State machine arm/disarm

Firmware hiện tại dùng state machine cơ bản:

```text
DISARMED
  ↓
ARMING
  ↓
ARMED_IDLE
  ↓
FLYING
```

Các trạng thái lỗi:

```text
FAILSAFE
SENSOR_ERROR
```

Ý nghĩa:

| State        | Ý nghĩa                                              |
| ------------ | ---------------------------------------------------- |
| DISARMED     | Motor dừng                                           |
| ARMING       | Đang giữ arm switch, chờ đủ thời gian                |
| ARMED_IDLE   | Đã arm, motor quay idle cố định                      |
| FLYING       | Throttle vượt ngưỡng, PID được phép điều khiển motor |
| FAILSAFE     | Mất SBUS hoặc frame lỗi, motor về 0                  |
| SENSOR_ERROR | Lỗi đọc cảm biến, motor về 0                         |

Thông số trong `board_config.h`:

```c
#define ARM_SWITCH_ON_THRESHOLD   0.35f
#define ARM_THROTTLE_LOW_MAX      0.06f
#define ARM_HOLD_TIME_MS          500

#define MOTOR_ARM_IDLE_POWER      0.045f
#define MOTOR_IDLE_POWER          0.050f

#define TAKEOFF_THROTTLE_START    0.14f
#define LAND_THROTTLE_BACK_IDLE   0.08f
```

Nếu motor không quay đều ở idle, có thể tăng nhẹ:

```c
#define MOTOR_ARM_IDLE_POWER      0.055f
#define MOTOR_IDLE_POWER          0.060f
```

---

## 6. Remap trục GY-85

Sau khi test thực tế, GY-85 cần remap trục trong `sensors.c`.

Mapping đang dùng:

```c
acc_body.x = -acc.x;
acc_body.y =  acc.y;
acc_body.z =  acc.z;

gyro_body.x =  gyro.x;
gyro_body.y = -gyro.y;
gyro_body.z =  gyro.z;
```

Kết quả đúng sau remap:

```text
Cúi đầu về trước:
roll  gần 0
pitch thay đổi mạnh

Nghiêng trái/phải:
roll thay đổi mạnh
pitch gần 0
```

Test đúng:

| Hành động        | Log đúng            | Motor phải tăng |
| ---------------- | ------------------- | --------------- |
| Cúi đầu về trước | Pitch đổi mạnh      | M2 + M4         |
| Ngửa đầu về sau  | Pitch đổi ngược lại | M1 + M3         |
| Nghiêng trái     | Roll đổi mạnh       | M1 + M2         |
| Nghiêng phải     | Roll đổi ngược lại  | M3 + M4         |

---

## 7. Mixer motor

Mixer hiện tại theo layout thực tế:

```c
motors->m1 = t + control->roll - control->pitch - control->yaw;  // M1 Rear-Left  CW
motors->m2 = t + control->roll + control->pitch + control->yaw;  // M2 Front-Left CCW
motors->m3 = t - control->roll - control->pitch + control->yaw;  // M3 Rear-Right CCW
motors->m4 = t - control->roll + control->pitch - control->yaw;  // M4 Front-Right CW
```

Giải thích:

```text
Pitch:
- Cúi đầu về trước  -> tăng M2 + M4
- Ngửa đầu về sau   -> tăng M1 + M3

Roll:
- Nghiêng trái      -> tăng M1 + M2
- Nghiêng phải      -> tăng M3 + M4

Yaw:
- Motor CCW: +yaw
- Motor CW : -yaw
```

---

## 8. Điều khiển PID

Firmware dùng cấu trúc cascade:

```text
Angle error
    ↓
Angle P
    ↓
Rate setpoint
    ↓
Rate PID
    ↓
Mixer
    ↓
Motor
```

Luồng trong code:

```c
roll_error_deg = setpoint->attitude.roll - state->attitude.roll;
roll_rate_sp = angle_kp * roll_error_deg;

roll_rate_error = roll_rate_sp - sensor->gyro.x;
control->roll = pid_update(&pid_roll, roll_rate_error, dt);
```

Thông số khởi đầu cho drone:

```text
Khối lượng: khoảng 190 g
Motor: 1404
Propeller: 3025
Motor distance: khoảng 100 mm
```

PID khởi đầu an toàn:

```c
void controller_pid_init(void)
{
    pid_axis_init(&pid_roll,  0.0022f, 0.00045f, 0.000012f, 0.06f);
    pid_axis_init(&pid_pitch, 0.0022f, 0.00045f, 0.000012f, 0.06f);
    pid_axis_init(&pid_yaw,   0.0010f, 0.00020f, 0.000000f, 0.04f);
}
```

Angle loop:

```c
const float angle_kp = 2.8f;
const float max_rate_rp = 120.0f;
```

Output limit khởi đầu:

```c
control->roll  = constrainf_local(pid_update(&pid_roll,  roll_rate_error,  dt), -0.08f, 0.08f);
control->pitch = constrainf_local(pid_update(&pid_pitch, pitch_rate_error, dt), -0.08f, 0.08f);
control->yaw   = constrainf_local(pid_update(&pid_yaw,   yaw_rate_error,   dt), -0.05f, 0.05f);
```

Nếu motor nóng hoặc rung nhanh, giảm D về 0:

```c
pid_axis_init(&pid_roll,  0.0022f, 0.00045f, 0.000000f, 0.06f);
pid_axis_init(&pid_pitch, 0.0022f, 0.00045f, 0.000000f, 0.06f);
```

---

## 9. Công thức PID

PID hiện tại có thể dùng dạng trapezoidal I-term:

```text
Output = Kp * Error
       + Ki * integral
       + Kd * derivative
```

I-term:

```text
integral += (Error + PrevError) * dt / 2
```

D-term:

```text
derivative = (Error - PrevError) / dt
```

Trong code có thêm lọc D-term để giảm nhiễu từ GY-85:

```c
const float d_alpha = 0.18f;
p->d_lpf += d_alpha * (derivative - p->d_lpf);
```

Lý do cần lọc D-term:

```text
- GY-85 dễ nhiễu khi motor quay
- D-term khuếch đại nhiễu cao tần
- Nếu D quá lớn, motor nóng và rung nhanh
```

---

## 10. ESC OneShot125-like

Firmware xuất ESC bằng LEDC:

```c
#define ESC_PWM_FREQ_HZ           2000
#define ESC_PULSE_MIN_US          125.0f
#define ESC_PULSE_MAX_US          250.0f
#define ESC_PULSE_STOP_US         125.0f
```

Ý nghĩa:

```text
power = 0.0 -> 125 us
power = 1.0 -> 250 us
PWM frequency = 2000 Hz
```

Khi disarm/failsafe, có thể kéo duty về 0 để ESC dừng hẳn:

```c
void esc_oneshot_stop_all(void)
{
    for (int i = 0; i < 4; i++) {
        ledc_set_duty(LEDC_MODE_USED, channels[i], 0);
        ledc_update_duty(LEDC_MODE_USED, channels[i]);
    }
}
```

---

## 11. Build và flash

Cấu hình target:

```bash
idf.py set-target esp32s3
```

Thêm dependency WS2812:

```bash
idf.py add-dependency "espressif/led_strip"
```

Build:

```bash
idf.py build
```

Flash và monitor:

```bash
idf.py -p COMx flash monitor
```

Ví dụ:

```bash
idf.py -p COM5 flash monitor
```

Nếu thay đổi nhiều file hoặc lỗi lạ:

```bash
idf.py fullclean
idf.py build
```

---

## 12. Cách test an toàn

### Bước 1: Test không gắn cánh

Trước khi cấp pin motor hoặc lắp cánh:

```text
- Kiểm tra GY-85 đọc đúng
- Kiểm tra SBUS nhận đúng
- Kiểm tra arm/disarm đúng
- Kiểm tra failsafe đúng
- Kiểm tra từng GPIO motor
```

### Bước 2: Test GPIO motor

Kết quả đúng hiện tại:

```text
GPIO4 -> M1 Rear-Left
GPIO5 -> M2 Front-Left
GPIO6 -> M3 Rear-Right
GPIO7 -> M4 Front-Right
```

### Bước 3: Test hướng phản ứng

Tháo cánh, arm idle, tăng throttle nhẹ để vào FLYING, sau đó nghiêng drone bằng tay.

Kết quả đúng:

| Hành động        | Motor phải tăng |
| ---------------- | --------------- |
| Cúi đầu về trước | M2 + M4         |
| Ngửa đầu về sau  | M1 + M3         |
| Nghiêng trái     | M1 + M2         |
| Nghiêng phải     | M3 + M4         |

Nếu motor tăng sai vị trí, kiểm tra lại:

```text
1. GPIO mapping trong board_config.h
2. Layout motor thực tế
3. Remap trục trong sensors.c
4. Mixer trong power_distribution.c
```

### Bước 4: Test failsafe

Tháo cánh, cho vào trạng thái FLYING, sau đó tắt transmitter hoặc ngắt SBUS.

Kết quả đúng:

```text
state=FAILSAFE
motor về 0
LED đỏ nháy nhanh
```

Nếu mất SBUS mà motor không về 0 thì không được bay thử.

---

## 13. Bay thử lần đầu

Chỉ bay thử khi đã đúng toàn bộ:

```text
- GPIO motor đúng
- Chiều quay motor đúng
- Cánh lắp đúng
- Cúi/ngửa/nghiêng thì motor phản ứng đúng
- Arm/disarm đúng
- Failsafe đúng
```

Cách bay thử:

```text
1. Chọn nơi rộng, không có người gần.
2. Đặt drone trên mặt phẳng.
3. Bật nguồn, không di chuyển drone khi calibrate.
4. Arm.
5. Tăng throttle rất chậm.
6. Chỉ nhấc lên 5–10 cm.
7. Nếu thấy lật/rung mạnh, disarm ngay.
```

Không nên bay cao hoặc bay xa ở lần đầu.

---

## 14. Dấu hiệu cần dừng test

Dừng ngay nếu gặp:

```text
- Vừa tăng ga là drone muốn lật
- Một bên motor tăng rất mạnh
- Motor nóng nhanh
- ESC nóng
- Drone rung nhanh
- Roll/pitch nhảy lung tung khi motor chạy
- Mất SBUS nhưng motor không dừng
- LED báo SENSOR_ERROR hoặc FAILSAFE
```

---

## 15. Cấu trúc thư mục

```text
main/
├── app_main.c
├── board_config.h
├── CMakeLists.txt
├── interface/
│   └── stabilizer_types.h
├── modules/
│   ├── system.c
│   ├── stabilizer.c
│   ├── sensors.c
│   ├── estimator_complementary.c
│   ├── commander_sbus.c
│   ├── controller_pid.c
│   └── power_distribution.c
└── drivers/
    ├── gy85.c
    ├── sbus.c
    ├── esc_oneshot_ledc.c
    └── status_led.c
```

---

## 16. Nâng cấp tiếp theo

Các bước có thể phát triển tiếp:

```text
1. Motor test mode qua UART
2. PID tuning qua UART
3. Battery monitor
4. VL53L1X altitude hold
5. Optical flow position hold
6. Blackbox logger
7. EKF/ESKF
8. Ground station telemetry
```

Lộ trình nâng cấp altitude hold:

```text
VL53L1X ToF
  -> height estimator
  -> height PID
  -> thrust correction
```

Lộ trình nâng cấp optical flow:

```text
Optical flow
  -> velocity x/y
  -> velocity PID
  -> desired roll/pitch
  -> attitude controller
```

---

## 17. Ghi chú bảo mật và an toàn

```text
- Dự án chỉ phục vụ học tập và thử nghiệm DIY.
- Không sử dụng gần người hoặc vật dễ hỏng.
- Luôn tháo cánh khi debug.
- Luôn test failsafe trước khi bay.
- Không copy PID từ drone khác mà không tune lại.
- Không bay khi chưa xác nhận đúng motor mapping.
```

---

## Author

Ma Văn Hùng

Project: ESP32-S3 Flight Controller Balance Firmware
