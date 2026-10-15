# FWMR — Robot Mecanum 4 bánh (STM32F401RE)

Firmware điều khiển robot 4 bánh mecanum, chạy trên board Nucleo-F401RE (STM32F401RET6, HSI 16 MHz), viết bằng STM32CubeIDE. Robot nhận vị trí **tuyệt đối** từ camera trần qua ESP-NOW, trộn với odometry (encoder + gyro) bằng một **EKF 4 trạng thái**, rồi tự chạy tới 1 điểm đích đặt trong `config.h` (go-to-point). Robot này mang **ID 29** trong hệ thống gateway nhiều robot.

## Phần cứng

- MCU: STM32F401RET6 (Nucleo-F401RE)
- 4 động cơ DC + driver kiểu TB6612FNG (mỗi bánh 2 chân DIR + 1 chân STBY chung)
- 4 encoder trục bánh (Timer Encoder Mode phần cứng, không cần ngắt), 1496 xung/vòng theo thông số nhà sản xuất
- IMU MPU6050 (I2C1) — gyro trục Z đo góc quay thân xe (yaw), cho EKF ước lượng cả bias
- UART6 (115200 baud, TX + RX, chân PC6/PC7): TX xuất telemetry; RX nhận vị trí tuyệt đối + lệnh từ camera trần
- `esp32C` — gateway đặt cạnh máy tính, nhận lệnh từ Python qua Serial rồi phát ESP-NOW tới đúng robot theo ID (dùng chung cho nhiều robot, không nằm trong repo này)
- `esp32R` — ESP32 gắn trên robot (MAC `14:33:5C:04:61:18`): chiều xuống chuyển tiếp nguyên văn gateway → STM32 PC7; chiều lên đọc telemetry từ PC6, in ra Serial Monitor của chính nó **và** gửi không dây sang `esp32Server`
- `esp32Server` — ESP32 cắm USB vào máy tính (MAC `94:E6:86:3A:C3:88`), chỉ nhận ESP-NOW từ `esp32R` rồi in lên Serial Monitor — cho phép xem log khi robot chạy không dây, không cần kéo dây USB theo robot
- 1 mã ArUco dán trên nóc robot, hướng đầu marker trùng hướng đầu robot

## Cấu trúc thư mục

```
Core/Inc, Core/Src          — toàn bộ code tự viết (driver + logic điều khiển)
Core/Startup                — startup assembly cho STM32F401RETx
esp32_vision/esp32_robot    — firmware esp32R: cầu ESP-NOW <-> UART, relay log (đang dùng)
esp32_vision/esp32_server_log — firmware esp32Server: nhận log qua ESP-NOW, in ra USB (đang dùng)
esp32_vision/pc_vision      — script phát pose giả để test khi chưa có camera
esp32_teleop/               — bản điều khiển bàn phím cũ, giữ làm lịch sử, không còn dùng
matlab_plot/                — script Octave/MATLAB vẽ lại log (fwmr_parse_log.m, plot_fwmr_runs.m, compare_enc_vs_imu.m) + log .txt + hình đã xuất
so-do-chan-FWMR.pdf         — sơ đồ chân hoàn chỉnh, dựng lại từ chính mã nguồn (06/10/2026)
tan_so_lay_mau_FWMR.xlsx    — bảng tần số lấy mẫu các bài đo
FWMR_run_test.ioc           — cấu hình STM32CubeMX (clock, v.v.) — KHÔNG đồng bộ với code thật, xem dưới
STM32F401RETX_*.ld          — linker script (Flash / RAM)
```

## Kiến trúc firmware

| Module | Vai trò |
|---|---|
| `config.h` | Hằng số cấu hình trung tâm: PWM, hình học robot, hệ số trượt, hiệu chuẩn động cơ, vòng giữ hướng, tham số camera, tham số 2 bài chạy (`enc_run_test`, `goto_test`), tham số EKF |
| `gpio.h/c` | Cấu hình chân GPIO điều khiển chiều quay (DIR) 4 động cơ + chân STBY driver |
| `motor.h/c` | Driver PWM (TIM1, 4 kênh, 20 kHz). `speed_to_duty()` dùng mô hình `duty = OFFSET[bánh] + K_FF·|ω|`; `motor_set_duty_raw()` ghi thẳng duty bỏ qua offset/K_FF, dùng cho bài đo duty chung |
| `encoder.h/c` | Đọc encoder 4 bánh bằng Timer Encoder Mode (TIM2/TIM3/TIM4/TIM5), xử lý tràn 16/32-bit, `ENC_SIGN` riêng từng bánh |
| `kinematics.h/c` | Động học thuận (FK) & nghịch (IK) cho mecanum 4 bánh (pseudo-inverse, bất biến qua các bài chạy) |
| `mpu6050.h/c` | Driver I2C1 mức thanh ghi cho MPU6050, hiệu chuẩn bias + tích phân gyro Z ra góc yaw, cho phép ghi đè yaw từ nguồn tuyệt đối |
| `odometry.h/c` | FK từ encoder → vận tốc thân xe, nhân hệ số trượt `K_SLIP_X/Y`, xoay theo yaw IMU, tích phân ra (x, y) trong vòng lặp chính; cho phép ghi đè (x, y) từ nguồn tuyệt đối |
| `pose_link.h/c` | Parse dòng lệnh từ gateway: pose tuyệt đối (xử lý), `STOP` (xử lý — `pose_link_take_stop()`), `START`/`WPLIST`/`WPCLR` (nhận biết, chưa xử lý); giữ pose mới nhất + watchdog + bộ đếm gói |
| `usart6.h/c` | TX: telemetry 115200 baud. RX: ngắt RXNE gom dòng vào **hàng đợi vòng 8 dòng** (PC7) |
| `ekf.h/c` | **EKF 4 trạng thái** [X, Y, θ, bias gyro] trộn camera (tuyệt đối, trễ, nhiễu) với encoder + gyro (50 Hz, trôi dần) — xem [Bộ lọc EKF](#bộ-lọc-ekf-ekfhc) |
| `goto_test.h/c` | Bài **go-to-point**: robot đặt ngẫu nhiên trong vùng camera, tự chạy tới 1 điểm đích bằng pose EKF — xem [Bài go-to-point](#bài-go-to-point-goto_testhc) |
| `enc_run_test.h/c` | Bài **chạy thẳng thuần encoder**: hiệu chuẩn tốc độ/lệch bánh, không camera — xem [Bài chạy thẳng thuần encoder](#bài-chạy-thẳng-thuần-encoder-enc_run_testhc) |
| `robot_control.h/c` | API cấp cao duy nhất: `robot_set_velocity(vx, vy, wz)` → tự IK + xuất PWM 4 bánh |
| `main.c` | Boot → hiệu chuẩn IMU → nếu có cờ bài test thì nhảy vào thẳng (không bao giờ quay lại) → vòng lặp 50 Hz: đọc pose + encoder + IMU → odometry → ghi đè pose khi có khung hình mới → vòng P-controller giữ hướng → `robot_set_velocity()` → log telemetry |

**Điểm thiết kế quan trọng:** toàn bộ driver ngoại vi được viết trực tiếp ở mức thanh ghi (`RCC->…`, `GPIOx->…`, `TIMx->…`, `I2Cx->…`), chỉ dùng HAL cho `HAL_Init()`/`HAL_GetTick()`/`HAL_Delay()`/SysTick. Vì vậy file `.ioc` **không** phản ánh đầy đủ cấu hình phần cứng thực tế — cẩn thận nếu mở lại CubeMX và generate code. Sơ đồ chân thật đã được dựng lại từ chính mã nguồn trong `so-do-chan-FWMR.pdf` (06/10/2026), đối chiếu khớp hoàn toàn với bộ nhớ project.

## Nguyên lý điều khiển

**Động học nghịch (IK)** — từ vận tốc thân xe mong muốn suy ra tốc độ góc từng bánh (pseudo-inverse, 4 bánh/3 bậc tự do):

```
ωFL = (vx − vy − K·ωz) / R      ωFR = (vx + vy + K·ωz) / R
ωRL = (vx + vy − K·ωz) / R      ωRR = (vx − vy + K·ωz) / R
```

**Động học thuận (FK)** — chiều ngược lại, dùng trong odometry và EKF:

```
vx = (R/4)·(ωFL + ωFR + ωRL + ωRR)
vy = (R/4)·(−ωFL + ωFR + ωRL − ωRR)
ωz = (R/4K)·(−ωFL + ωFR − ωRL + ωRR)
```

**Duty PWM (feed-forward + offset bù ma sát tĩnh):** `duty = OFFSET[bánh] + K_FF · |ω|`, mỗi bánh một `OFFSET` riêng (`MOTOR_DUTY_OFFSET_FL/FR/RL/RR`). Bảng offset đo **không tải** (23–24/09/2026) không còn đúng ở duty cao/có tải: bài chạy 2.4 m thuần encoder (06–07/10/2026) cho thấy bánh **RR yếu hẳn** so với 3 bánh còn lại (chậm hơn RL 7–10% ở cùng duty), một đặc điểm cơ khí của riêng cụm RR, không phải do bù sai — xem [Bài chạy thẳng thuần encoder](#bài-chạy-thẳng-thuần-encoder-enc_run_testhc).

**Vòng hiệu chỉnh hướng (P-controller), dùng trong vòng lặp chính:** yaw đo từ MPU6050 so với `THETA_TARGET_DEG`, sai số nhân `HEADING_KP` (giới hạn ±`HEADING_MAX_WZ`) rồi cộng thẳng vào lệnh `ωz`. `goto_test.c` và `enc_run_test.c` có vòng giữ hướng riêng, tham số khác (xem từng bài).

**Odometry (`odometry.c`):** vận tốc thân xe lấy từ FK trên delta-encoder, nhân hệ số trượt `K_SLIP_X`/`K_SLIP_Y` (bù quãng đường thật nhỏ hơn quãng đường bánh lăn — ước lượng tạm 0.97/0.75, chưa đo riêng từng trục), xoay theo yaw IMU, tích phân Euler ra (x, y). Nguồn tuyệt đối từ camera ghi đè trực tiếp qua `odometry_set_pose()`/`mpu6050_set_yaw_deg()` trong vòng lặp chính — cách trộn "thô" này chỉ dùng khi **không** chạy `goto_test`/`enc_run_test`; hai bài đó tự quản lý pose riêng (EKF hoặc odometry thuần).

## Vòng lặp điều khiển chính

> Chỉ chạy tới đây khi cả 2 cờ bài test (`ENC_RUN_TEST_ENABLE`, `GOTO_TEST_ENABLE`) đều = 0 trong `config.h` — xem cảnh báo ở mục [Bài go-to-point](#bài-go-to-point-goto_testhc) về cờ đang bật hiện tại. `main.c` có `#error` nếu bật cả 2 cờ cùng lúc.

1. `pose_link_poll()` — vét hàng đợi các dòng đã về qua ngắt UART6 RX.
2. Đọc delta-encoder 4 bánh + cập nhật IMU → `odometry_update()`.
3. Nếu có khung hình camera **mới**: `odometry_set_pose()` và `mpu6050_set_yaw_deg()` ghi đè vị trí/hướng. Sai lệch góc được chuẩn hoá về [−180°, 180°] trước khi trộn, vì yaw gyro cộng dồn không bị giới hạn còn θ camera thì có.
4. `target_vx_world`/`target_vy_world` — **mặc định 0** (robot đứng yên, chỉ tự giữ hướng).
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
| `STOP` | Dừng khẩn | **Đã xử lý** (15/10/2026) — `goto_test_run()` kiểm tra `pose_link_take_stop()` mỗi chu kỳ, dừng động cơ ngay |
| `START` | Cho phép chạy | Nhận biết, chưa xử lý |
| `WPLIST;<x1>;<y1>;<x2>;<y2>;…` | Danh sách điểm đích | Nhận biết, chưa xử lý |
| `WPCLR` | Xoá danh sách điểm đích | Nhận biết, chưa xử lý |

**Quy ước đơn vị** (firmware tự quy đổi khi parse): `x`, `y` tính bằng **centimet** đổi sang mét; `theta` tính bằng **radian** đổi sang độ. Chiều dương của θ phải là **ngược chiều kim đồng hồ nhìn từ trên xuống**, trùng chiều dương gyro Z — đã kiểm chứng trên phần cứng 24/09/2026, lệch 0.1° sau khi xoay 90°.

**Không có checksum trên đường truyền.** Gateway gửi text thuần, không magic byte, không CRC. Hai lớp chặn duy nhất: esp32R lọc ký tự không in được, và `pose_link` từ chối giá trị ngoài `POSE_XY_ABS_MAX_CM` hoặc θ ngoài ±7 rad. `goto_test.c` thêm một lớp nữa ở tầng ứng dụng: nếu marker bị che, PC có thể gửi lại pose **cũ giống hệt** — lúc robot đang di chuyển thì 2 khung giống nhau tuyệt đối là bất thường, nên bị bỏ qua thay vì đưa vào EKF.

**Hàng đợi RX.** Gateway kết thúc gói bằng `#` và không gửi ký tự xuống dòng, nên `usart6.c` coi cả `\n`, `\r` và `#` là hết dòng. RX giữ hàng đợi vòng 8 dòng × 128 byte; bộ đếm `drop` trong dòng `PLINK` cho biết có dòng nào bị rơi vì hàng đợi đầy không.

**Relay log không dây (mới, 06/10/2026).** `esp32R` không chỉ chuyển lệnh xuống mà còn đọc telemetry STM32 gửi lên (PC6), in ra Serial Monitor của chính nó **và** gửi tiếp bằng ESP-NOW sang `esp32Server` (một ESP32 khác cắm USB vào PC, sketch `esp32_server_log.ino`) — nhờ vậy xem được log đầy đủ khi robot chạy không dây, không cần kéo dây USB theo robot. Để tránh ngập màn hình (STM32 bắn ~100 dòng/giây), `esp32R` lọc bớt: dòng `IMU,` tắt hẳn theo mặc định (`LOG_SHOW_IMU=0`), dòng `POSE,` chỉ in 1/10 (`LOG_POSE_EVERY_N=10`), các dòng khác (`PLINK`, `ENC`, `GOTO:`, `GT,`, khối kết quả, `STM32:...`) in đủ.

Tham số phía STM32 trong `config.h`: `POSE_LINK_ENABLE`, `POSE_LINK_TIMEOUT_MS`, `POSE_XY_ABS_MAX_CM`, `POSE_YAW_ALPHA`, `POSE_LOG_EVERY_N`.

## Bộ lọc EKF (`ekf.h/c`)

EKF 4 trạng thái **s = [X, Y, θ, b]** (m, m, rad, rad/s — hệ camera G), viết mới 15/10/2026 để trộn camera (tuyệt đối, có trễ, có nhiễu, có thể mất khung) với encoder+gyro (50 Hz, không trễ nhưng trôi dần). Thiết kế đầy đủ nằm trong doc dự án `dan_y_ekf_go_to_point_15102026.md`; phần đã lên firmware:

- **Dự đoán (`ekf_predict`), mỗi 20 ms:** nhận độ dời thân xe từ FK-encoder (đã bù `K_SLIP_X/Y`) và gyro Z thô; bias `b` được **trừ ra khỏi gyro trước khi tích phân** (`ω = gz − b`), rồi cộng dồn (X, Y, θ) bằng phép xoay theo góc trung điểm — đúng công thức odometry đang dùng. Hiệp phương sai `P` lan truyền qua Jacobian 4×4 đầy đủ, cộng nhiễu quá trình `Q` ước lượng từ tỉ lệ quãng đường đi được (`EKF_Q_ENC_X/Y`) và nhiễu/trôi gyro (`EKF_Q_GYRO_DPS`, `EKF_Q_BIAS_DPS`).
- **Bù trễ camera:** giữ lịch sử trạng thái 1.28 s (64 mẫu × 20 ms, `HIST_N`). Khi khung camera về, nó được so với trạng thái **tại đúng thời điểm chụp** (`t_meas_ms = now − EKF_CAM_DELAY_MS`, trễ ước lượng 200 ms, tra trong lịch sử bằng `hist_find`), không phải trạng thái hiện tại — nếu không bù, ở 0.2–0.3 m/s mỗi 0.2 s trễ gây sai vài cm.
- **Cập nhật (`ekf_update_cam`):** đo z = [Xc, Yc, θc], H = [I₃ 0], R = diag(`EKF_R_XY_M²`, `EKF_R_XY_M²`, `EKF_R_TH_DEG²`) — `EKF_R_TH_DEG = 0.53°` lấy từ số đo nhiễu camera thực tế 24/09/2026, `EKF_R_XY_M` còn là ước lượng tạm. **Cổng Mahalanobis** (`d² > EKF_GATE = 16.3`, χ² 3 bậc tự do 99.9%) loại khung sai lệch bất thường (nhận nhầm marker, nhiễu gai) mà không đưa vào bộ lọc. Loại liên tiếp quá `EKF_REJECT_RESET = 10` khung thì coi là bộ lọc đã lạc hướng — nạp lại toàn bộ trạng thái từ khung camera mới nhất (`ekf_set_pose`, giữ nguyên bias đã học).
- **Sau mỗi cập nhật**, hiệu chỉnh (ΔX, ΔY, Δθ) được cộng dồn vào **cả lịch sử** — nếu không, khung kế tiếp sẽ so với trạng thái cũ và cộng lại đúng hiệu chỉnh này lần nữa.
- `P` được đối xứng hoá (`symmetrize()`) sau mỗi bước dự đoán/cập nhật để chặn trôi số học tích lũy qua hàng nghìn chu kỳ float.
- Expose: `ekf_x/y/th/bias()`, `ekf_sigma_xy()` (độ lệch chuẩn vị trí ước lượng), `ekf_n_ok/reject/reset()` (đếm chẩn đoán).

EKF hiện chạy **song song làm nguồn vị trí chính cho `goto_test.c`**; chưa chạy độc lập để so sánh offline với log thô (bước 4–5 trong lộ trình thiết kế vẫn chưa làm — xem [Trạng thái](#trạng-thái--hướng-phát-triển-tiếp-theo)).

## Bài go-to-point (`goto_test.h/c`)

> **Cấu hình hiện tại trong `config.h`: `GOTO_TEST_ENABLE = 1`, `ENC_RUN_TEST_ENABLE = 0`** — nạp firmware như đang có trong repo sẽ **không** chạy vòng điều khiển bình thường mà nhảy thẳng vào bài go-to-point, đích `(x=1.80 m; y=0.60 m)` trên hệ camera G (`GOTO_GOAL_X_CM/Y_CM`), `GOTO_FACE_GOAL = 1` (xoay tại chỗ hướng mũi xe về đích trước khi chạy). **Nếu MPU6050 không được tìm thấy lúc boot, bài test treo lại và không chạy động cơ** (kiểm tra an toàn trong `goto_test_run`). Đặt `GOTO_TEST_ENABLE = 0` rồi nạp lại khi muốn quay về vòng điều khiển bình thường.

Robot đặt **ngẫu nhiên** trong vùng camera nhìn thấy (không cần đặt tại gốc toạ độ như bản cũ), hướng ban đầu tuỳ ý — bài test tự lấy điểm xuất phát từ trung bình `GOTO_INIT_FRAMES` khung camera.

**Máy trạng thái:** `SETTLE → (ROTATE) → DRIVE → SETTLE → ... → DONE`, có `WAIT_CAM` xen vào khi mất camera giữa lúc đang chạy.

1. **Chờ camera + chuẩn hoá điểm xuất phát:** lấy trung bình `GOTO_INIT_FRAMES` khung camera làm (x₀, y₀, θ₀), khởi tạo EKF (`ekf_init`).
2. **Đếm ngược `GOTO_START_DELAY_MS`, xe đứng yên:** vẫn chạy `sense()` (predict + update EKF) mỗi chu kỳ — đây là lúc EKF học bias gyro khi còn thấy camera liên tục, trước khi xe di chuyển.
3. **SETTLE:** chờ `GOTO_SETTLE_MS`, lấy trung bình camera trong `GOTO_SETTLE_AVG_MS` cuối; nếu EKF lệch trung bình đó hơn `GOTO_RESNAP_M`/`GOTO_RESNAP_DEG` thì nạp lại pose EKF theo camera (`n_resnap` đếm số lần). Rồi so sánh với đích: còn xa hơn `GOTO_POS_TOL_M` → qua ROTATE/DRIVE; đủ gần nhưng lệch hướng hơn `GOTO_TH_TOL_DEG` → ROTATE chỉnh hướng cuối; cả hai đạt → DONE (kết quả DAT). Thử lại tối đa `GOTO_MAX_TRIES` lần.
4. **ROTATE** (chỉ khi `GOTO_FACE_GOAL` và lệch hướng hơn `GOTO_TH_GO_DEG`): quay tại chỗ với `wz` tỉ lệ sai số góc, kẹp trong [`GOTO_WZ_MIN`, `GOTO_WZ_MAX`] (`GOTO_WZ_MIN = 0.9` rad/s để vượt ngưỡng khởi động bánh khi quay tại chỗ — nhắc lại phát hiện 24/09/2026 rằng `HEADING_MAX_WZ = 0.5` không đủ để tự xoay được). Dừng khi đủ gần, vừa vượt qua (đổi dấu sai số), hoặc quá `GOTO_ROTATE_MAX_MS`.
5. **DRIVE:** lệnh vận tốc theo hướng thẳng tới đích trên hệ thân xe (chia `K_SLIP_X/Y` trước khi gửi, vì lệnh là độ dời *mong muốn*, ngược với odometry là *đo được*), `v = GOTO_KP_POS · dist` kẹp `GOTO_V_MAX`, ramp gia tốc `GOTO_ACCEL`, có **sàn tốc độ theo hướng đi** `v_floor_for()` (như bản cũ, nhưng nay còn bỏ qua cặp bánh gần như không tải khi đi chéo 45° thay vì luôn lấy cặp yếu nhất). Dừng khi vào `GOTO_STOP_M` hoặc đi xa dần hơn điểm gần nhất đã từng đạt + `GOTO_OVERSHOOT_M` (phát hiện đã đi quá, không chờ tới khi lệch hẳn). Vòng giữ hướng riêng (`GOTO_KP_TH_DRIVE`, kẹp `GOTO_WZ_MAX_DRIVE`).
6. **WAIT_CAM:** nếu đang di chuyển mà quá `GOTO_CAM_LOST_MS` không có khung camera mới hợp lệ → dừng động cơ, chờ camera quay lại rồi mới về SETTLE chạy tiếp. **Lưu ý:** đây là "dừng và chờ", KHÁC với thiết kế "hầm" đã bàn (chạy tiếp bằng odometry/EKF-dự đoán tối đa N giây) — hành vi hầm thật vẫn chưa được lập trình, xem roadmap.
7. **Kết thúc:** dừng động cơ, chờ `GOTO_SETTLE_MS` cho xe hết trôi + camera hết trễ, lấy trung bình camera làm số liệu "sự thật" độc lập với EKF, in khối `===== GOTO KET QUA =====` lặp lại mỗi 5 giây — gồm kết quả (DAT/CHUA_DAT/HET_GIO/DUNG_BOI_STOP), toạ độ xuất phát/đích/cuối theo cả camera và EKF, sai số thật theo camera, số lần thử, và chẩn đoán EKF (bias học được, số khung nhận/loại/nạp lại, số lần resnap).

**Dừng khẩn:** nhận được `STOP` qua gateway (`pose_link_take_stop()`), hoặc quá `GOTO_TIMEOUT_MS` (60 s) kể từ lúc bắt đầu chạy.

**Log:** `GOTO:...` (sự kiện đổi trạng thái, có toạ độ EKF lúc đó), `GT,t_ms,trang_thai,x_mm,y_mm,th_deg,xc_mm,yc_mm,thc_deg,dist_mm,loi_huong_deg,v` mỗi `GOTO_PRINT_EVERY_MS` (x,y,th = EKF; xc,yc,thc = khung camera thô cuối — so sánh trực tiếp được 2 nguồn), khối kết quả ở trên.

## Bài chạy thẳng thuần encoder (`enc_run_test.h/c`)

Bài hiệu chuẩn **không dùng camera, không giữ hướng bắt buộc** — mục đích đo lệch tự nhiên giữa 4 bánh và đặc tính duty→tốc độ, tách khỏi mọi sai số do camera/EKF gây ra. Dừng khi 1 bánh bất kỳ đạt `ENC_RUN_TARGET_COUNTS` xung (ứng với `ENC_RUN_DIST_M`, hiện 3.0 m).

Hai chế độ, chọn bằng `ENC_RUN_EQUAL_DUTY`:
- **`= 0`** — chạy theo vận tốc đặt (`ENC_RUN_V`) qua `robot_set_velocity()` như bình thường (có offset + K_FF), ramp từ `ROBOT_V_MIN`, tuỳ chọn giữ hướng bằng θ tính từ encoder (`ENC_RUN_HEADING_HOLD`).
- **`= 1`** (đang bật) — cấp **cùng 1 duty thô** (`ENC_RUN_DUTY`, bỏ qua offset/K_FF) cho cả 4 bánh qua `motor_set_duty_raw()`, để xem độ lệch *tự nhiên* giữa 4 động cơ. Có thể bật thêm vòng giữ hướng bằng IMU kiểu PI+D vi sai duty (`ENC_RUN_IMU_HOLD`): `u = KP·e + ∫KI·e − KD·gz` (e = sai số yaw so với 0°), rồi `FL,RL = duty − u`, `FR,RR = duty + u`.

**Kết quả đo được (06–07/10/2026, 2 đợt, xem doc dự án để đủ số liệu):**
- Đợt 1 (duty = offset + K_FF·ω, ramp 300 ms): θ cuối dao động −3° đến −25° giữa các lần chạy — **feed-forward thuần không lặp lại được**, một phần do bảng offset (đo không tải) tạo ra lệch "cặp chéo" giả (RL được cộng dư, RR bị thiếu).
- Đợt 2 (duty chung 400, bỏ offset/K_FF): lặp lại tốt hơn nhiều. FL, FR, RL lệch ≤ 2% lẫn nhau; **RR luôn chậm nhất, 7–10% so với RL** ở cùng duty — đặc điểm cơ khí riêng của cụm RR (động cơ/hộp số/driver/dây), không phải do bù offset sai. RR còn có hiệu ứng "nóng máy": nhanh dần qua các lần chạy liên tiếp (90.4% → 93.1% so với RL).
- Duty 400 (50% PWM_MAX) cho xe chạy thẳng ~0.50 m/s.

**Log:** `ENC,t_ms,duty|v_cmd,FL,FR,RL,RR,x_mm,y_mm,th_deg,yaw_deg,gz_dps,u,xi_mm,yi_mm` mỗi `ENC_RUN_LOG_EVERY_MS` (ngừng in khi gần chạm mốc để không trễ việc kiểm tra dừng — `ENC_RUN_LOG_GUARD`), `ENCRUN:CHAM_MOC`/`HET_GIO` lúc dừng, khối `===== KET QUA =====` lặp lại mỗi 5 giây — gồm xung lúc chạm mốc và lúc dừng hẳn, quãng đường mỗi bánh (mm), toạ độ theo odometry thuần và theo odometry+hướng IMU, `yaw_max_abs`, duty hiệu chỉnh trung bình lúc ổn định, vận tốc trung bình.

## Số liệu hiệu chuẩn đã chốt

| Đại lượng | Giá trị | Ghi chú |
|---|---|---|
| `COUNTS_PER_REV` | 1496 xung/vòng | theo thông số nhà sản xuất (06/10/2026; trước đó dùng 1500, đo thực tế ~1497–1499, lệch <0.15%) |
| Ngưỡng PWM khởi động (không tải) | FL 90, FR 115, RL 125, RR 115 | 23/09/2026 — **không còn đúng ở duty cao/có tải**, xem bài chạy thẳng thuần encoder |
| `K_FF`, `MOTOR_DUTY_OFFSET_*` | 23.3; FL 21, FR 46, RL 56, RR 46 | `offset = ngưỡng − 69`; vẫn dùng cho vòng điều khiển chính và go-to-point |
| Bánh RR yếu hơn cấu trúc | chậm hơn RL 7–10% ở cùng duty | 06–07/10/2026, đo bằng duty chung — chưa sửa trong firmware |
| `MOTOR_OMEGA_MIN` / `ROBOT_V_MIN` | 2.97 rad/s / 0.102 m/s | cận trên, chưa đo thấp hơn |
| Dấu θ camera so với gyro | cùng chiều, lệch 0.1° sau 90° | 24/09/2026, 0° camera trùng trục X |
| Nhiễu θ camera khi đứng yên | σ = 0.53° (`EKF_R_TH_DEG`) | dùng trực tiếp làm R cho EKF |
| `K_SLIP_X` / `K_SLIP_Y` | 0.97 / 0.75 | ước lượng tạm từ 1 bài chạy chéo (28/09/2026), **chưa đo riêng từng trục** — rủi ro chính khi mất camera |
| Độ trễ camera ước lượng | 200 ms (`EKF_CAM_DELAY_MS`) | ước lượng từ bài goto 28/09/2026, **chưa đo lại cho đúng** |

## Thêm một bài chạy mới

Hạ tầng đã sẵn: sau `odometry_reset()` trong `main.c`, pose, yaw, encoder và kênh camera đều chạy. Hai cách:
- Bài chạy dạng **đặt vận tốc**: thay `target_vx_world` / `target_vy_world` trong vòng lặp chính, giữ nguyên vòng giữ hướng và telemetry.
- Bài chạy **chiếm toàn quyền** (có máy trạng thái, log riêng, như `goto_test.c`/`enc_run_test.c`): viết vào file riêng `xxx_test.c/h`, gọi từ `main.c` ngay trước `STM32:RUNNING`, bật/tắt bằng 1 cờ trong `config.h`. Nhớ thêm `#error` chặn bật 2 cờ bài test cùng lúc, theo khuôn `ENC_RUN_TEST_ENABLE`/`GOTO_TEST_ENABLE` hiện có.

## Cách test khi chưa có camera

1. **Chỉ STM32, không cần ESP32 nào:** nối USB-TTL vào PC7, gõ tay `100;200;0.78540#`. Dòng `POSE,...` phải nhảy về `1.000, 2.000, 45.00`. Hoặc chạy `python esp32_vision/pc_vision/pose_sim.py COM<x>` để phát liên tục (`--mode static|circle|step`).
2. **Cả chuỗi không dây:** nạp `esp32_vision/esp32_robot/esp32_robot.ino` cho esp32R, rồi `python esp32_vision/pc_vision/pose_sim.py COM<x> --target gateway` để bắn qua gateway. Kiểm tra `PLINK,...`: `ok` tăng đều, `err` và `drop` đứng yên.

## Build & nạp code

1. Cài [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html).
2. Clone repo, import vào STM32CubeIDE (`File → Open Projects from File System…`).
3. **Đóng tab `config.h`/`main.c`/file test đang sửa trước khi build**, hoặc bật Window → Preferences → General → Workspace → "Refresh using native hooks or polling" — IDE có lỗi ghi buffer cũ đè file mới nếu tab còn mở khi sửa file từ ngoài IDE, đã từng làm mất cả buổi debug sai.
4. Build rồi nạp qua ST-Link.
5. Mở Serial Monitor của `esp32R` (hoặc `esp32Server` nếu chạy không dây) để xem `STM32:BOOT`, `GOTO:...`/`ENCRUN:...`, `GT,...`/`ENC,...`.

## Trạng thái & hướng phát triển tiếp theo

Đã xong: driver PWM/encoder/IMU/UART (TX + RX có hàng đợi), động học IK/FK, vòng giữ hướng bằng gyro, kênh nhận vị trí tuyệt đối từ camera trần qua gateway ESP-NOW (kể cả `STOP`), relay log không dây qua `esp32Server`, hiệu chuẩn `K_FF`/offset PWM + phát hiện bánh RR yếu về cấu trúc, **EKF 4 trạng thái** (camera + encoder + gyro, bù trễ camera, cổng Mahalanobis, tự nạp lại khi lạc hướng) đã viết và nối vào go-to-point, **bài go-to-point** chạy từ vị trí xuất phát bất kỳ với máy trạng thái xoay-rồi-chạy + thử lại + resnap khi đứng yên, bài chạy thẳng thuần encoder để tách bạch sai số cơ khí khỏi sai số camera/EKF.

Đang thiếu / dự kiến làm tiếp (theo lộ trình EKF 15/10/2026 và "đi từ điểm A tới điểm B, có hầm trên đường"):
- Chạy đánh giá go-to-point hiện tại (không hầm) trước — đã lên cờ `GOTO_TEST_ENABLE=1`, đích (1.80; 0.60) m, cần đọc log để đánh giá sai số/độ ổn định trước khi làm bước tiếp.
- Đo độ trễ camera τ thật (hiện `EKF_CAM_DELAY_MS=200` chỉ là ước lượng) và nhiễu σx/σy của camera (`EKF_R_XY_M` cũng mới là ước lượng).
- Chạy EKF offline trong MATLAB trên log thô để chỉnh Q/R/τ trước khi tin số liệu firmware (bước 3–4 trong doc thiết kế, chưa làm).
- Cài đúng hành vi "hầm": hiện `WAIT_CAM` chỉ dừng-và-chờ camera quay lại; thiết kế đã chốt là chạy tiếp bằng dự đoán EKF tối đa N giây rồi mới dừng — chưa lên code.
- Đo riêng `K_SLIP_X` (chạy thẳng trục X) và `K_SLIP_Y` (chạy ngang trục Y) — hiện vẫn dùng số ước lượng từ 1 bài chạy chéo.
- Giải quyết bánh RR yếu hơn cấu trúc: kiểm tra cơ khí hoặc thêm vòng PID tốc độ riêng từng bánh theo encoder (hiện vẫn feed-forward + offset tĩnh).
- Xử lý `START`/`WPLIST`/`WPCLR`: cho phép chạy có điều kiện và mở rộng go-to-point sang nhiều điểm liên tiếp.
- Đồng bộ lại `FWMR_run_test.ioc` với cấu hình phần cứng thực tế (đã có sơ đồ chân đúng trong `so-do-chan-FWMR.pdf` để đối chiếu).
