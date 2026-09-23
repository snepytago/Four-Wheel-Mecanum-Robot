# FWMR — Robot Mecanum 4 bánh (STM32F401RE)

Firmware điều khiển robot 4 bánh mecanum, chạy trên board Nucleo-F401RE (STM32F401RET6, 84 MHz), viết bằng STM32CubeIDE. Robot nhận lệnh vận tốc thân xe `(vx, vy, ωz)`, tự tính động học nghịch ra tốc độ 4 bánh, ước lượng vị trí bằng odometry (fusion encoder + IMU), tự hiệu chỉnh hướng đi bằng gyro, và nhận vị trí **tuyệt đối** từ camera trần qua đường ESP-NOW. Robot này mang **ID 29** trong hệ thống gateway nhiều robot.

## Phần cứng

- MCU: STM32F401RET6 (Nucleo-F401RE)
- 4 động cơ DC + driver kiểu TB6612FNG (mỗi bánh 2 chân DIR + 1 chân STBY chung)
- 4 encoder trục bánh (đọc bằng Timer Encoder Mode phần cứng, không cần ngắt)
- IMU MPU6050 (I2C1) — dùng gyro trục Z để đo góc quay thân xe (yaw)
- UART6 (115200 baud, TX + RX): TX xuất telemetry ra máy tính để log/vẽ đồ thị; RX (chân PC7) nhận vị trí tuyệt đối từ camera trần
- `esp32C` — gateway đặt cạnh máy tính, nhận lệnh từ Python qua Serial rồi phát ESP-NOW tới đúng robot theo ID (dùng chung cho nhiều robot, không nằm trong repo này)
- `esp32R` — ESP32 gắn trên robot, MAC `14:33:5C:04:61:18`, nhận ESP-NOW rồi đẩy xuống UART của STM32
- 1 mã ArUco dán trên nóc robot, hướng đầu marker trùng hướng đầu robot

## Cấu trúc thư mục

```
Core/Inc, Core/Src       — toàn bộ code tự viết (driver + logic điều khiển)
Core/Startup             — startup assembly cho STM32F401RETx
esp32_vision/esp32_robot — firmware esp32R: cầu ESP-NOW -> UART (đang dùng)
esp32_vision/pc_vision   — script phát pose giả để test khi chưa có camera
esp32_teleop/            — bản điều khiển bàn phím cũ, giữ làm lịch sử, không còn dùng
FWMR_run_test.ioc        — cấu hình STM32CubeMX (clock, v.v.)
STM32F401RETX_*.ld       — linker script (Flash / RAM)
```

## Kiến trúc firmware

| Module | Vai trò |
|---|---|
| `config.h` | Hằng số cấu hình trung tâm: PWM, hình học robot, hệ số hiệu chỉnh, chu kỳ điều khiển, tham số vòng giữ hướng, tham số nguồn pose camera |
| `gpio.h/c` | Cấu hình chân GPIO điều khiển chiều quay (DIR) 4 động cơ + chân STBY driver |
| `motor.h/c` | Driver PWM (TIM1, 4 kênh, ~105 kHz) + logic set chiều quay/tốc độ từng bánh |
| `encoder.h/c` | Đọc encoder 4 bánh bằng Timer Encoder Mode (TIM2/TIM3/TIM4/TIM5), xử lý tràn 16/32-bit |
| `kinematics.h/c` | Động học thuận (FK) & nghịch (IK) cho mecanum 4 bánh |
| `mpu6050.h/c` | Driver I2C1 mức thanh ghi cho MPU6050, hiệu chuẩn bias + tích phân gyro Z ra góc yaw, cho phép ghi đè yaw từ nguồn tuyệt đối |
| `odometry.h/c` | Ước lượng vị trí (x, y): FK từ encoder cho vận tốc, góc θ lấy trực tiếp từ IMU; cho phép ghi đè (x, y) từ nguồn tuyệt đối |
| `pose_link.h/c` | Parse dòng lệnh nhận được từ gateway, giữ pose mới nhất + watchdog + bộ đếm gói theo từng loại |
| `usart6.h/c` | TX: telemetry 115200 baud. RX: ngắt RXNE gom dòng vào **hàng đợi vòng 8 dòng** (PC7) |
| `pwm_test.h/c` | Bài đo ngưỡng PWM khởi động 4 bánh (2 pha) — xem [Đo ngưỡng PWM khởi động](#đo-ngưỡng-pwm-khởi-động-4-bánh) |
| `robot_control.h/c` | API cấp cao duy nhất: `robot_set_velocity(vx, vy, wz)` → tự IK + xuất PWM 4 bánh |
| `main.c` | Boot → hiệu chuẩn IMU → vòng lặp 50 Hz: đọc pose + encoder + IMU → odometry → ghi đè pose khi có khung hình mới → vòng P-controller giữ hướng → `robot_set_velocity()` → log telemetry |

**Điểm thiết kế quan trọng:** toàn bộ driver ngoại vi được viết trực tiếp ở mức thanh ghi (`RCC->…`, `GPIOx->…`, `TIMx->…`, `I2Cx->…`), chỉ dùng HAL cho `HAL_Init()`/`HAL_GetTick()`/`HAL_Delay()`/SysTick. Vì vậy file `.ioc` **không** phản ánh đầy đủ cấu hình phần cứng thực tế — cẩn thận nếu mở lại CubeMX và generate code.

## Nguyên lý điều khiển

**Động học nghịch (IK)** — từ vận tốc thân xe mong muốn suy ra tốc độ góc từng bánh:

```
ωFL = (vx − vy − K·ωz) / R      ωFR = (vx + vy + K·ωz) / R
ωRL = (vx + vy − K·ωz) / R      ωRR = (vx − vy + K·ωz) / R
```

**Động học thuận (FK)** — chiều ngược lại, dùng trong odometry:

```
vx = (R/4)·(ωFL + ωFR + ωRL + ωRR)
vy = (R/4)·(−ωFL + ωFR + ωRL − ωRR)
ωz = (R/4K)·(−ωFL + ωFR − ωRL + ωRR)
```

**Vòng hiệu chỉnh hướng (P-controller):** yaw đo từ MPU6050 so với `THETA_TARGET_DEG`, sai số nhân `HEADING_KP` (giới hạn ±`HEADING_MAX_WZ`) rồi cộng thẳng vào lệnh `ωz`.

**Odometry (fusion encoder + IMU + camera):** vận tốc thân xe lấy từ FK trên encoder, góc θ lấy từ IMU, và mỗi khi có khung hình mới từ camera thì `(x, y, θ)` được ghi đè bằng giá trị tuyệt đối đo được. Giữa hai khung hình, encoder + gyro tiếp tục tích phân để giữ nhịp 50 Hz — camera sửa sai số tích luỹ, odometry lấp khoảng trống giữa các khung và giữ robot không bị mù khi marker tạm bị che.

## Vòng lặp điều khiển chính

> Chỉ chạy tới đây khi cả `PWM_TEST_ENABLE` và `PWM_TEST2_ENABLE` đều = 0 trong `config.h` — xem cảnh báo ở mục [Đo ngưỡng PWM khởi động](#đo-ngưỡng-pwm-khởi-động-4-bánh) về trạng thái hiện tại.

1. `pose_link_poll()` — vét hàng đợi các dòng đã về qua ngắt UART6 RX.
2. Đọc delta-encoder 4 bánh + cập nhật IMU → `odometry_update()`.
3. Nếu có khung hình camera **mới**: `odometry_set_pose()` và `mpu6050_set_yaw_deg()` ghi đè vị trí/hướng. Sai lệch góc được chuẩn hoá về [−180°, 180°] trước khi trộn, vì yaw gyro cộng dồn không bị giới hạn còn θ camera thì có.
4. `target_vx_world`/`target_vy_world` — **hiện mặc định 0** (robot đứng yên, chỉ tự giữ hướng). Vòng go-to-point sẽ thay vào ở bước kế tiếp.
5. Xoay sang hệ thân xe theo θ hiện tại, cộng `wz_cmd` từ vòng giữ hướng, gọi `robot_set_velocity()`.
6. Gửi telemetry `IMU,...`, `POSE,x,y,theta` và `PLINK,...` (giãn tần số) qua UART6 TX.

## Định vị tuyệt đối bằng camera trần

```
Camera trần --ảnh--> PC (OpenCV + ArUco)
   --Serial "29;<x_cm>;<y_cm>;<theta_rad>#"--> esp32C (gateway)
        gateway cắt "29;" rồi gửi phần còn lại tới MAC của robot 29
   --ESP-NOW--> esp32R --UART 115200 + '\n'--> STM32 PC7 --> pose_link.c
```

**Các dòng lệnh gateway có thể gửi tới robot:**

| Dòng | Ý nghĩa | Trạng thái |
|---|---|---|
| `<x_cm>;<y_cm>;<theta_rad>` | Pose tuyệt đối | Đã xử lý |
| `START` / `STOP` | Cho phép / dừng chạy | Nhận biết, chưa xử lý |
| `WPLIST;<x1>;<y1>;<x2>;<y2>;…` | Danh sách điểm đích | Nhận biết, chưa xử lý |
| `WPCLR` | Xoá danh sách điểm đích | Nhận biết, chưa xử lý |

**Quy ước đơn vị** (firmware tự quy đổi khi parse): `x`, `y` tính bằng **centimet** đổi sang mét; `theta` tính bằng **radian** đổi sang độ. Chiều dương của θ phải là **ngược chiều kim đồng hồ nhìn từ trên xuống**, trùng chiều dương gyro Z. Trục y của ảnh camera hướng xuống nên `atan2` tính thẳng trên toạ độ ảnh sẽ ngược dấu — phía PC phải đảo dấu. Kiểm chứng bằng tay trước khi tin số liệu: xoay robot 90° ngược chiều kim đồng hồ, `PLINK` phải báo θ **tăng** khoảng 90.

**Không có checksum trên đường truyền.** Gateway gửi text thuần, không magic byte, không CRC. Hai lớp chặn duy nhất: esp32R lọc ký tự không in được, và `pose_link` từ chối giá trị ngoài `POSE_XY_ABS_MAX_CM` hoặc θ ngoài ±7 rad.

**Hàng đợi RX.** Gateway kết thúc gói bằng `#` và không gửi ký tự xuống dòng, nên `usart6.c` coi cả `\n`, `\r` và `#` là hết dòng — nhờ vậy có thể cắm thẳng gateway vào PC7 để thử, bỏ qua esp32R. RX giữ hàng đợi vòng 8 dòng × 128 byte vì các dòng về theo cụm trong vài ms trong khi main loop chỉ đọc 50 lần/giây; bộ đếm `drop` trong dòng `PLINK` cho biết có dòng nào bị rơi vì hàng đợi đầy không.

Tham số phía STM32 trong `config.h`: `POSE_LINK_ENABLE`, `POSE_LINK_TIMEOUT_MS`, `POSE_XY_ABS_MAX_CM`, `POSE_YAW_ALPHA`, `POSE_LOG_EVERY_N`.

## Đo ngưỡng PWM khởi động 4 bánh

Feed-forward `K_FF` hiện là tuyến tính thuần (`duty = K_FF · |speed|`), nhưng thực tế mỗi bánh cần một **duty tối thiểu** để thắng ma sát tĩnh trước khi bắt đầu quay — dưới ngưỡng đó động cơ chỉ "è è" mà không nhúc nhích. Vòng go-to-point sau này sẽ bị "chết cứng" cách đích vài cm nếu không biết con số này để đặt `V_MIN`. `pwm_test.c` đo ngưỡng này theo 2 pha, bật/tắt bằng `PWM_TEST_ENABLE`/`PWM_TEST2_ENABLE` trong `config.h` (chỉ bật **1 trong 2** cùng lúc):

- **Pha 1** (`pwm_test_run`, không tải — bánh kê lên khỏi mặt sàn): tăng dần duty từng bánh một, dùng encoder làm trọng tài ("đã quay" = đủ `PWM_TEST_MIN_COUNTS` xung liên tục `PWM_TEST_CONFIRM_STEPS` nấc). Chỉ dùng để so sánh 4 động cơ với nhau (bánh nào lệch hẳn = hộp số kẹt/dây lỏng/driver yếu), **không** dùng số đo được làm `V_MIN` vì thiếu ma sát sàn và quán tính robot thật.
- **Pha 2** (`pwm_test2_run`, trên sàn thật — cả 4 bánh cùng duty, đi tới/lui xen kẽ): trọng tài là **camera** (qua `pose_link`), không phải encoder, vì ở duty thấp có bánh đã quay trong khi bánh khác còn kẹt — robot không dịch chuyển dù encoder vẫn ghi nhận có chuyển động (bánh quay trượt tại chỗ). Cần gateway + camera đang chạy (có pose hợp lệ) trước khi đo. Encoder vẫn được ghi song song — chênh lệch quãng đường bánh lăn được so với quãng đường robot thật đo bằng camera chính là **độ trượt** của bánh mecanum trên mặt sàn.

Cả hai pha chạy **một lần lúc boot rồi treo lại** in kết quả lặp định kỳ — **không bao giờ** rơi xuống vòng điều khiển bình thường bên dưới. Log dạng `PWMT,<bánh>,<duty>,<số_xung>,<rad/s>` (mỗi nấc), `PWMTH,<bánh>,<duty_ngưỡng>` (đã chốt, pha 1) hoặc `PWMT2,<duty>,<F|B>,<d_cam_mm>,<v_cam>,<v_enc>,<trượt_%>` / `PWMTH2,<duty_ngưỡng>,<v_cam_tại_ngưỡng>` (pha 2).

> **Cấu hình hiện tại trong `config.h`: `PWM_TEST_ENABLE = 0`, `PWM_TEST2_ENABLE = 1`** — nạp firmware như đang có trong repo sẽ chạy **bài đo pha 2**, không phải vòng điều khiển bình thường (robot sẽ tự đi tới/lui từng nấc duty, không giữ yên/giữ hướng như mô tả ở các mục trên). Nhớ đặt `PWM_TEST2_ENABLE = 0` rồi nạp lại khi muốn quay về vòng điều khiển bình thường.

## Cách test khi chưa có camera

1. **Chỉ STM32, không cần ESP32 nào:** nối USB-TTL vào PC7, gõ tay `100;200;0.78540#`. Dòng `POSE,...` phải nhảy về `1.000, 2.000, 45.00`. Hoặc chạy `python esp32_vision/pc_vision/pose_sim.py COM<x>` để phát liên tục (`--mode static|circle|step`).
2. **Cả chuỗi không dây:** nạp `esp32_vision/esp32_robot/esp32_robot.ino` cho esp32R, rồi `python esp32_vision/pc_vision/pose_sim.py COM<x> --target gateway` để bắn qua gateway. Kiểm tra `PLINK,...`: `ok` tăng đều, `err` và `drop` đứng yên.

## Build & nạp code

1. Cài [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html).
2. Clone repo, import vào STM32CubeIDE (`File → Open Projects from File System…`).
3. Build rồi nạp qua ST-Link.
4. Mở terminal UART6 (115200-8-N-1) để xem `STM32:BOOT`, `POSE,...`, `PLINK,...`.

## Trạng thái & hướng phát triển tiếp theo

Đã xong: driver PWM/encoder/IMU/UART (TX + RX có hàng đợi), động học IK/FK, odometry fusion, vòng giữ hướng bằng gyro, hiệu chỉnh tĩnh `K_FF` (66.0 → 29.3, chưa verify lại lần 2), kênh nhận vị trí tuyệt đối từ camera trần qua gateway ESP-NOW, và bài đo ngưỡng PWM khởi động 4 bánh (2 pha, xem mục trên).

Đang thiếu / dự kiến làm tiếp:
- Đặt `V_MIN` trong vòng điều khiển từ kết quả đo ngưỡng PWM (pha 2), hiện mới có bài đo chứ chưa đưa số vào `speed_to_duty()`.
- Đối chiếu thực nghiệm gốc toạ độ, chiều trục và dấu θ giữa camera và firmware.
- Xử lý `START`/`STOP`: cờ cho phép chạy trong vòng điều khiển.
- Xử lý `WPLIST`/`WPCLR` + vòng go-to-point bám lần lượt từng điểm đích, kèm điều kiện dừng khi mất pose.
- Vòng phản hồi tốc độ (PID) theo encoder cho từng bánh — hiện chỉ là feed-forward tuyến tính (`K_FF`).
- Đồng bộ lại `FWMR_run_test.ioc` với cấu hình phần cứng thực tế.
