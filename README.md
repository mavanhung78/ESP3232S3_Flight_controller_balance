# ESP32-S3 Crazyflie-style Balance Firmware

Mục tiêu của phiên bản này: tạo cấu trúc giống firmware Crazyflie ở mức đơn giản:

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

Phiên bản hiện tại chỉ làm bước đầu tiên: **cân bằng roll/pitch bằng accelerometer + gyroscope của GY-85**, nhận lệnh **SBUS**, xuất 4 motor bằng tín hiệu **OneShot125-like** qua LEDC.

> ⚠️ Test lần đầu phải tháo cánh quạt. Không cấp pin motor khi chưa kiểm tra chiều gyro, chiều motor và chiều mixer.

---

## 1. Phần cứng mặc định

| Khối | Linh kiện | ESP32-S3 pin mặc định |
|---|---|---|
| Accel | ADXL345 trên GY-85 | I2C SDA GPIO8, SCL GPIO9 |
| Gyro | ITG3205 trên GY-85 | I2C SDA GPIO8, SCL GPIO9 |
| SBUS RX | Receiver SBUS | UART1 RX GPIO16 |
| ESC M1 | ESC / motor 1 | GPIO10 |
| ESC M2 | ESC / motor 2 | GPIO11 |
| ESC M3 | ESC / motor 3 | GPIO12 |
| ESC M4 | ESC / motor 4 | GPIO13 |

Sửa pin tại:

```c
main/board_config.h
```

---

## 2. Kênh SBUS mặc định

| SBUS channel | Chức năng |
|---|---|
| CH1 | Roll |
| CH2 | Pitch |
| CH3 | Throttle |
| CH4 | Yaw rate |
| CH5 | Arm switch |

SBUS thường là tín hiệu đảo mức. Code đã bật:

```c
#define SBUS_RX_INVERT 1
```

Nếu receiver của bạn đã xuất SBUS uninverted, đổi thành:

```c
#define SBUS_RX_INVERT 0
```

---

## 3. Build và flash

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

Thay `COMx` bằng cổng của bạn, ví dụ `COM5`.

---

## 4. Luồng điều khiển

```text
GY-85 accel/gyro
    ↓
sensors_acquire()
    ↓
estimator_complementary_update()
    ↓
state: roll, pitch, yaw, gyro rate
    ↓
commander_get_setpoint() từ SBUS
    ↓
controller_pid_update()
    ↓
control: thrust, roll, pitch, yaw
    ↓
power_distribution_mix()
    ↓
motor m1, m2, m3, m4
    ↓
esc_oneshot_write_all()
```

---

## 5. Cách test an toàn

### Bước 1: chưa gắn cánh

Mở monitor, đặt drone nằm yên. Log phải có roll/pitch gần 0 độ:

```text
roll=0.5 pitch=-1.0 gyro=[0.2 -0.1 0.0]
```

Nếu để yên mà gyro lớn, kiểm tra lại calibration hoặc sensor.

### Bước 2: kiểm tra chiều roll/pitch

Nghiêng drone sang phải. Roll phải đổi đúng chiều mong muốn. Nếu ngược, sửa dấu trong:

```c
main/modules/sensors.c
```

Ví dụ:

```c
sensor->gyro.x = -(gyro.x - gyro_offset.x);
sensor->acc.x = -acc.x;
```

### Bước 3: kiểm tra mixer

Arm nhưng vẫn tháo cánh. Nghiêng drone bằng tay:

- Nghiêng phải thì motor bên phải nên tăng/giảm theo hướng chống lại nghiêng.
- Nếu drone tự làm nghiêng nặng hơn, đảo dấu `control->roll` hoặc đổi thứ tự motor trong `power_distribution.c`.

### Bước 4: tuning PID

File tuning chính:

```c
main/modules/controller_pid.c
```

Gains mặc định đang để rất nhẹ:

```c
pid_roll:  kp=0.0040, ki=0.0015, kd=0.000035
pid_pitch: kp=0.0040, ki=0.0015, kd=0.000035
pid_yaw:   kp=0.0022, ki=0.0008, kd=0.000000
```

Thứ tự chỉnh:

1. Đặt `ki = 0`, `kd = 0`, tăng `kp` đến khi phản ứng đủ nhanh nhưng chưa rung.
2. Thêm `kd` nhỏ để giảm rung.
3. Thêm `ki` rất nhỏ để giảm lệch lâu dài.

---

## 6. Ghi chú quan trọng

- Đây chưa phải flight controller hoàn chỉnh.
- Chưa có ToF giữ độ cao.
- Chưa có optical flow giữ vị trí.
- Chưa có yaw ổn định bằng magnetometer.
- Chưa có battery compensation.
- Chưa có blackbox/logger.
- OneShot125 đang tạo bằng LEDC 2 kHz, xung 125-250 us. Một số ESC có thể cần calibration hoặc đổi protocol.

---

## 7. Nâng cấp tiếp theo

Sau khi roll/pitch cân bằng ổn:

```text
VL53L1X ToF
  -> height estimator
  -> height PID
  -> thrust correction
```

Sau đó:

```text
Optical flow
  -> velocity x/y
  -> velocity PID
  -> desired roll/pitch
```
