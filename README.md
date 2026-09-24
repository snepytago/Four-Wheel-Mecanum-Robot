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
| `motor.h/c` | Driver PWM (TIM1, 4 kênh, ~105 kHz) + logic set chiều quay/tốc độ từng bánh. `speed_to_duty()` dùng mô hình `duty = OFFSET[bánh] + K_FF·|ω|` — mỗi bánh một hằng số offset riêng, hiệu chỉnh từ bài đo ngưỡng PWM |
| `encoder.h/c` | Đọc encoder 4 bánh bằng Timer Encoder Mode (TIM2/TIM3/TIM4/TIM5), xử lý tràn 16/32-bit |
| `kinematics.h/c` | Động học thuận (FK) & nghịch (IK) cho mecanum 4 bánh |
| `mpu6050.h/c` | Driver I2C1 mức thanh ghi cho MPU6050, hiệu chuẩn bias + tích phân gyro Z ra góc yaw, cho phép ghi đè yaw từ nguồn tuyệt đối |
| `odometry.h/c` | Ước lượng vị trí (x, y): FK từ encoder cho vận tốc, góc θ lấy trực tiếp từ IMU; cho phép ghi đè (x, y) từ nguồn tuyệt đối |
| `pose_link.h/c` | Parse dòng lệnh nhận được từ gateway, giữ pose mới nhất + watchdog + bộ đếm gói theo từng loại |
| `usart6.h/c` | TX: telemetry 115200 baud. RX: ngắt RXNE gom dòng vào **hàng đợi vòng 8 dòng** (PC7) |
| `pwm_test.h/c` | 4 bài đo/kiểm tra độc lập, mỗi bài chiếm toàn quyền lúc boot rồi treo lại (không rơi xuống vòng điều khiển bình thường) — xem [Các bài đo & kiểm tra hiệu chuẩn](#các-bài-đo--kiểm-tra-hiệu-chuẩn) |
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

**Duty PWM (feed-forward + offset bù ma sát tĩnh):** `duty = OFFSET[bánh] + K_FF · |ω|`, mỗi bánh một `OFFSET` riêng (`MOTOR_DUTY_OFFSET_FL/FR/RL/RR` trong `config.h`). Bản cũ chỉ nhân `K_FF · |ω|` (không offset) sai về nguyên tắc: ở tốc độ thấp duty rơi xuống dưới ngưỡng ma sát tĩnh nên bánh đứng im, còn ở tốc độ cao lại cấp thừa — hai sai số ngược chiều nên chỉnh riêng `K_FF` không bao giờ sửa được cả dải tốc độ. `K_FF = 23.3` và các `OFFSET` hiện tại đến từ 4 bài đo thực tế ngày 23–24/09/2026 (xem [Các bài đo & kiểm tra hiệu chuẩn](#các-bài-đo--kiểm-tra-hiệu-chuẩn)).

**Vòng hiệu chỉnh hướng (P-controller):** yaw đo từ MPU6050 so với `THETA_TARGET_DEG`, sai số nhân `HEADING_KP` (giới hạn ±`HEADING_MAX_WZ`) rồi cộng thẳng vào lệnh `ωz`.

**Odometry (fusion encoder + IMU + camera):** vận tốc thân xe lấy từ FK trên encoder, góc θ lấy từ IMU, và mỗi khi có khung hình mới từ camera thì `(x, y, θ)` được ghi đè bằng giá trị tuyệt đối đo được. Giữa hai khung hình, encoder + gyro tiếp tục tích phân để giữ nhịp 50 Hz — camera sửa sai số tích luỹ, odometry lấp khoảng trống giữa các khung và giữ robot không bị mù khi marker tạm bị che.

## Vòng lặp điều khiển chính

> Chỉ chạy tới đây khi cả 4 cờ bài đo (`PWM_TEST_ENABLE`, `PWM_TEST2_ENABLE`, `POSE_CHECK_ENABLE`, `STRAIGHT_TEST_ENABLE`) đều = 0 trong `config.h` — xem cảnh báo ở mục [Các bài đo & kiểm tra hiệu chuẩn](#các-bài-đo--kiểm-tra-hiệu-chuẩn) về trạng thái hiện tại.

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

## Các bài đo & kiểm tra hiệu chuẩn

`pwm_test.c` gom 4 bài đo/kiểm tra độc lập, mỗi bài bật bằng 1 cờ riêng trong `config.h` (`PWM_TEST_ENABLE`, `PWM_TEST2_ENABLE`, `POSE_CHECK_ENABLE`, `STRAIGHT_TEST_ENABLE`) — **chỉ nên bật 1 cờ tại một thời điểm**. Cả 4 đều chạy **một lần lúc boot rồi treo lại** in kết quả lặp định kỳ, **không bao giờ** rơi xuống vòng điều khiển bình thường.

**1. Pha 1 — ngưỡng PWM không tải** (`pwm_test_run`, `PWM_TEST_ENABLE`): bánh kê lên khỏi mặt sàn, tăng dần duty từng bánh một, dùng encoder làm trọng tài. Chỉ dùng để so sánh 4 động cơ với nhau (bánh lệch hẳn = hộp số kẹt/dây lỏng/driver yếu), **không** dùng số đo được làm `V_MIN` vì thiếu ma sát sàn và quán tính robot thật. Log: `PWMT,<bánh>,<duty>,<số_xung>,<rad/s>`, `PWMTH,<bánh>,<duty_ngưỡng>`, `PWMSUM,<FL>,<FR>,<RL>,<RR>`.

**Kết quả đo được (23/09/2026), đã đưa vào `config.h`:** ngưỡng khởi động không tải FL=90, FR=115, RL=125, RR=115 (lệch tới 39% giữa các bánh).

**2. Pha 2 — trên sàn thật, camera làm trọng tài** (`pwm_test2_run`, `PWM_TEST2_ENABLE`): cả 4 bánh cùng duty, đi tới/lui xen kẽ. Trọng tài là **camera** (qua `pose_link`), không phải encoder — ở duty thấp có bánh đã quay trong khi bánh khác còn kẹt, robot không dịch chuyển dù encoder vẫn ghi nhận có chuyển động (bánh quay trượt tại chỗ). Cần gateway + camera đang chạy (có pose hợp lệ). Encoder vẫn ghi song song — chênh lệch quãng đường bánh lăn được so với quãng đường robot thật đo bằng camera chính là **độ trượt** bánh mecanum trên sàn. `PWM_TEST2_USE_OFFSET=1`: mỗi bánh cộng thêm offset riêng đã đo ở pha 1 (`PWM_TH_FL/FR/RL/RR`) rồi mới quét chung — vừa đo vừa thử luôn cách bù vùng chết. Log: `PWMT2,<duty>,<F|B>,<d_cam_mm>,<v_cam>,<v_enc>,<trượt_%>,<dtheta_deg>,<x_mm>,<y_mm>,<dFL>,<dFR>,<dRL>,<dRR>`, `PWMTH2,<duty_ngưỡng>,<v_cam_tại_ngưỡng>`.

**3. Kiểm tra dấu θ camera** (`pose_theta_check_run`, `POSE_CHECK_ENABLE`): không chạy động cơ, chỉ in song song θ từ camera và yaw từ gyro trong lúc xoay robot bằng tay, để xác nhận hai nguồn cùng chiều dương trước khi tin số liệu ghi đè lẫn nhau. Log: `THCHK,<valid>,<th_cam>,<yaw_gyro>,<d_cam>,<d_gyro>`, kết luận in ra khi đã xoay đủ 30°.

**Đã chạy 24/09/2026 — kết luận: camera và gyro CÙNG CHIỀU**, lệch nhau 0.1° sau khi xoay 90° (quy ước đúng, giữ nguyên). Độ nhiễu θ camera lúc đứng yên: lệch chuẩn 0.53° (dùng làm R cho Kalman filter sau này nếu cần).

**4. Chạy thẳng — kiểm chứng K_FF, vòng giữ hướng, độ trễ camera** (`straight_test_run`, `STRAIGHT_TEST_ENABLE`): đi qua đúng đường vận hành thật (odometry → ghi đè pose → vòng giữ hướng → `robot_set_velocity()`, không ghi thẳng PWM như 2 bài trên), tăng tốc dần lên `ST_TARGET_V` (0.15 m/s, trên `ROBOT_V_MIN`), giữ vài giây, rồi cắt lệnh đột ngột và tiếp tục theo dõi để đo độ trễ camera. Cần gateway + camera đang chạy và >1.5m khoảng trống phía trước. Log: `ST,<t_ms>,<v_cmd>,<x_mm>,<y_mm>,<th_cam>,<yaw>,<th_err>,<d_enc_mm>,<d_cam_mm>`, `STSUM,...`.

> **Cấu hình hiện tại trong `config.h`: `STRAIGHT_TEST_ENABLE = 1`, 3 cờ còn lại = 0** — nạp firmware như đang có trong repo sẽ chạy **bài chạy thẳng** (mục 4), không phải vòng điều khiển bình thường. Đặt cả 4 cờ về 0 rồi nạp lại khi muốn quay về vòng điều khiển bình thường (đứng yên + tự giữ hướng).

## Cách test khi chưa có camera

1. **Chỉ STM32, không cần ESP32 nào:** nối USB-TTL vào PC7, gõ tay `100;200;0.78540#`. Dòng `POSE,...` phải nhảy về `1.000, 2.000, 45.00`. Hoặc chạy `python esp32_vision/pc_vision/pose_sim.py COM<x>` để phát liên tục (`--mode static|circle|step`).
2. **Cả chuỗi không dây:** nạp `esp32_vision/esp32_robot/esp32_robot.ino` cho esp32R, rồi `python esp32_vision/pc_vision/pose_sim.py COM<x> --target gateway` để bắn qua gateway. Kiểm tra `PLINK,...`: `ok` tăng đều, `err` và `drop` đứng yên.

## Build & nạp code

1. Cài [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html).
2. Clone repo, import vào STM32CubeIDE (`File → Open Projects from File System…`).
3. Build rồi nạp qua ST-Link.
4. Mở terminal UART6 (115200-8-N-1) để xem `STM32:BOOT`, `POSE,...`, `PLINK,...`.

## Trạng thái & hướng phát triển tiếp theo

Đã xong: driver PWM/encoder/IMU/UART (TX + RX có hàng đợi), động học IK/FK, odometry fusion, vòng giữ hướng bằng gyro, kênh nhận vị trí tuyệt đối từ camera trần qua gateway ESP-NOW, bộ 4 bài đo/kiểm tra hiệu chuẩn (`pwm_test.c`), hiệu chuẩn `K_FF=23.3` + offset PWM riêng từng bánh đã đưa vào `motor.c` (từ dữ liệu đo thật 23–24/09/2026), và đối chiếu xác nhận quy ước dấu θ giữa camera và gyro (cùng chiều, đúng).

Đang thiếu / dự kiến làm tiếp:
- Đọc kết quả bài chạy thẳng (`straight_test_run`, đang bật) để xác nhận `K_FF`/offset mới đã đủ chính xác, hoặc hiệu chỉnh thêm.
- Đưa `ROBOT_V_MIN`/`MOTOR_OMEGA_MIN` vào vòng go-to-point khi viết (hiện mới là hằng số ghi lại cận trên, chưa có chỗ nào enforce).
- Xử lý `START`/`STOP`: cờ cho phép chạy trong vòng điều khiển.
- Xử lý `WPLIST`/`WPCLR` + vòng go-to-point bám lần lượt từng điểm đích, kèm điều kiện dừng khi mất pose.
- Vòng phản hồi tốc độ (PID) theo encoder cho từng bánh — hiện chỉ là feed-forward + offset tĩnh.
- Đồng bộ lại `FWMR_run_test.ioc` với cấu hình phần cứng thực tế.
