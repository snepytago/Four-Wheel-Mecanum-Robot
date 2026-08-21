# FWMR — Robot Mecanum 4 bánh (STM32F401RE)

Firmware điều khiển robot 4 bánh mecanum, chạy trên board Nucleo-F401RE (STM32F401RET6, 84 MHz), viết bằng STM32CubeIDE. Robot nhận lệnh vận tốc thân xe `(vx, vy, ωz)`, tự tính động học nghịch ra tốc độ 4 bánh, đồng thời ước lượng vị trí bằng odometry (fusion encoder + IMU) và tự hiệu chỉnh hướng đi bằng gyro.

## Phần cứng

- MCU: STM32F401RET6 (Nucleo-F401RE)
- 4 động cơ DC + driver kiểu TB6612FNG (mỗi bánh 2 chân DIR + 1 chân STBY chung)
- 4 encoder trục bánh (đọc bằng Timer Encoder Mode phần cứng, không cần ngắt)
- IMU MPU6050 (I2C1) — dùng gyro trục Z để đo góc quay thân xe (yaw)
- UART6 (115200 baud, TX only) — xuất telemetry ra máy tính để log/vẽ đồ thị

## Cấu trúc thư mục

```
Core/Inc, Core/Src     — toàn bộ code tự viết (driver + logic điều khiển)
Core/Startup           — startup assembly cho STM32F401RETx
FWMR_run_test.ioc      — cấu hình STM32CubeMX (clock, v.v.)
STM32F401RETX_*.ld     — linker script (Flash / RAM)
.project, .cproject, .mxproject, .settings/ — file dự án STM32CubeIDE
```

## Kiến trúc firmware

Mỗi khối chức năng là một cặp `.h/.c` độc lập trong `Core/Inc` và `Core/Src`:

| Module | Vai trò |
|---|---|
| `config.h` | Hằng số cấu hình trung tâm: PWM, hình học robot, hệ số hiệu chỉnh, chu kỳ điều khiển, tham số các bài test |
| `gpio.h/c` | Cấu hình chân GPIO điều khiển chiều quay (DIR) 4 động cơ + chân STBY driver |
| `motor.h/c` | Driver PWM (TIM1, 4 kênh, ~105 kHz) + logic set chiều quay/tốc độ từng bánh |
| `encoder.h/c` | Đọc encoder 4 bánh bằng Timer Encoder Mode (TIM2/TIM3/TIM4/TIM5), xử lý tràn 16/32-bit |
| `kinematics.h/c` | Động học thuận (FK) & nghịch (IK) cho mecanum 4 bánh |
| `mpu6050.h/c` | Driver I2C1 mức thanh ghi cho MPU6050, hiệu chuẩn bias + tích phân gyro Z ra góc yaw |
| `odometry.h/c` | Ước lượng vị trí (x, y): FK từ encoder cho vận tốc, góc θ lấy trực tiếp từ IMU |
| `usart6.h/c` | Gửi telemetry qua UART6 (115200 baud), có hàm gửi số nguyên/số thực tự viết |
| `robot_control.h/c` | API cấp cao duy nhất: `robot_set_velocity(vx, vy, wz)` → tự IK + xuất PWM 4 bánh |
| `main.c` | Kịch bản test: boot → hiệu chuẩn IMU → chạy theo quỹ đạo đặt sẵn → log telemetry → tự dừng |

**Điểm thiết kế quan trọng:** toàn bộ driver ngoại vi (GPIO, Timer PWM, Timer Encoder, I2C, USART) được viết trực tiếp ở mức thanh ghi (`RCC->…`, `GPIOx->…`, `TIMx->…`, `I2Cx->…`) để kiểm soát chặt timing, chỉ dùng HAL cho `HAL_Init()`/`HAL_GetTick()`/`HAL_Delay()`/SysTick. Vì vậy file `.ioc` hiện **không** phản ánh đầy đủ cấu hình phần cứng thực tế — cần cẩn thận nếu mở lại CubeMX và generate code, tránh bị ghi đè phần cấu hình thủ công.

## Nguyên lý điều khiển

**Động học nghịch (IK)** — từ vận tốc thân xe mong muốn suy ra tốc độ góc từng bánh, dùng ma trận giả nghịch đảo (hệ 4 bánh nhưng chỉ 3 bậc tự do):

```
ωFL = (vx − vy − K·ωz) / R      ωFR = (vx + vy + K·ωz) / R
ωRL = (vx + vy − K·ωz) / R      ωRR = (vx − vy + K·ωz) / R
```

**Động học thuận (FK)** — chiều ngược lại, dùng trong odometry để ước lượng vận tốc thân xe thực tế từ encoder:

```
vx = (R/4)·(ωFL + ωFR + ωRL + ωRR)
vy = (R/4)·(−ωFL + ωFR + ωRL − ωRR)
ωz = (R/4K)·(−ωFL + ωFR − ωRL + ωRR)
```

Với `R = WHEEL_R` (bán kính bánh) và `K = K_GEOM` (nửa dài + nửa rộng khung xe), cả hai đều khai báo trong `config.h`.

**Vòng hiệu chỉnh hướng (P-controller):** góc yaw đo từ MPU6050 được so sánh với góc mục tiêu `THETA_TARGET_DEG`, sai số nhân với `HEADING_KP` (giới hạn ±`HEADING_MAX_WZ`) rồi cộng thẳng vào lệnh `ωz` — robot vừa di chuyển vừa tự chỉnh hướng, không cần dừng lại.

**Odometry (fusion encoder + IMU):** vận tốc thân xe lấy từ FK trên encoder, còn góc hướng θ lấy trực tiếp từ IMU (không tích phân lại lần hai) — khai thác đúng thế mạnh từng cảm biến: encoder cho quãng đường, IMU cho góc quay (ít trôi hơn khi bánh trượt).

## Kịch bản test hiện tại (`main.c`)

`main.c` hiện chạy bài test **hình vuông** (`TEST_SQUARE_ENABLE` trong `config.h`): robot đi liên tiếp 4 cạnh hình vuông cạnh `SQUARE_SIDE_M`, **giữ nguyên hướng thân xe** suốt quá trình, chỉ đổi vector vận tốc tịnh tiến ở mỗi góc — khai thác đúng khả năng holonomic của mecanum (bánh vi sai/Ackermann không làm được điều này). Ngoài ra còn 2 bài test khác chọn bằng compile switch trong `config.h`:

- `TEST_ARC_ENABLE` — đi theo cung tròn bán kính `TEST_ARC_R1_M`, giữ nguyên hướng thân xe.
- Mặc định (tắt cả hai switch trên) — chạy thẳng theo `TARGET_VX_WORLD`/`TARGET_VY_WORLD` tới khi đạt `TARGET_DIST_M`.

Trong lúc chạy, mỗi 20 ms (`CONTROL_DT_MS`) firmware gửi telemetry qua UART6 dạng CSV: dòng `IMU,...`, `POSE,x,y,theta`, và tuỳ bài test có thêm `SQ,...` hoặc `ARC_REF,...` — dùng để log bằng PuTTY/Tera Term rồi vẽ đồ thị (Excel/Python) so sánh quỹ đạo lý thuyết và thực tế.

## Build & nạp code

1. Cài [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html).
2. Clone repo này.
3. Import project vào STM32CubeIDE (`File → Open Projects from File System…`, chọn thư mục repo).
4. Build (búa) rồi nạp vào board qua ST-Link (nút Debug/Run).
5. Mở terminal UART6 (PuTTY/Tera Term, 115200-8-N-1) để xem log `STM32:BOOT`, `POSE,...`, v.v.

## Trạng thái & hướng phát triển tiếp theo

Đã xong: driver PWM/encoder/IMU/UART, động học IK/FK, odometry fusion, vòng hiệu chỉnh hướng bằng gyro, 3 kịch bản test (thẳng, cung tròn, hình vuông), hiệu chỉnh tĩnh hệ số feed-forward `K_FF` từ dữ liệu thực đo.

Đang thiếu / dự kiến làm tiếp:
- Vòng phản hồi tốc độ (PID) theo encoder cho từng bánh — hiện chỉ là feed-forward tuyến tính (`K_FF`), nhạy với tải/ma sát/mức pin.
- Kênh nhận lệnh điều khiển từ ngoài (UART6 hiện chỉ có chiều gửi).
- Đồng bộ lại `FWMR_run_test.ioc` với cấu hình phần cứng thực tế đang chạy thủ công trong code.
