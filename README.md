# FWMR — Robot Mecanum 4 bánh (STM32F401RE)

Firmware điều khiển robot 4 bánh mecanum, chạy trên board Nucleo-F401RE (STM32F401RET6, HSI 16 MHz), viết bằng STM32CubeIDE. Robot nhận vị trí **tuyệt đối** từ camera trần qua ESP-NOW, trộn với odometry (encoder + gyro) bằng một **EKF 4 trạng thái**. Từ 19/10/2026 firmware ở trạng thái **hạ tầng sạch**: các bài test cũ (chạy thẳng thuần encoder, encoder + IMU, EKF, go-to-point) đã chuyển vào `tests_luu_tru/`, `main.c` chỉ còn vòng điều khiển 50 Hz chờ gắn bài test mới. 20/10/2026 thêm 3 bài test mới cho **chương zigzag**: `strafe_cal_test`, `rot_cal_test` (hiệu chuẩn), `zigzag_test` (bài chính, 19 bước đi + quay đầu + về). **Đang bật: `ZIGZAG_TEST_ENABLE 1`, `ZZ_MODE 1` (encoder + IMU), `ZZ_STEP_LAST 19` (chạy hết).** Robot này mang **ID 29** trong hệ thống gateway nhiều robot.

## Phần cứng

- MCU: STM32F401RET6 (Nucleo-F401RE)
- 4 động cơ DC + driver kiểu TB6612FNG (mỗi bánh 2 chân DIR + 1 chân STBY chung)
- 4 encoder trục bánh (Timer Encoder Mode phần cứng, không cần ngắt), 1496 xung/vòng theo thông số nhà sản xuất
- IMU MPU6050 (I2C1) — gyro trục Z đo góc quay thân xe (yaw), cho EKF ước lượng cả bias
- UART6 (115200 baud, TX + RX, chân PC6/PC7): TX xuất telemetry; RX nhận vị trí tuyệt đối + lệnh từ camera trần
- `esp32C` — gateway đặt cạnh máy tính, nhận lệnh từ Python qua Serial rồi phát ESP-NOW tới đúng robot theo ID (dùng chung cho nhiều robot, không nằm trong repo này)
- `esp32R` — ESP32 gắn trên robot (MAC `14:33:5C:04:61:18`): chiều xuống chuyển tiếp nguyên văn gateway → STM32 PC7; chiều lên đọc telemetry từ PC6, in ra Serial Monitor của chính nó **và** gửi không dây sang `esp32Server`
- `esp32Server` — ESP32 cắm USB vào laptop (MAC `1C:69:20:A4:D0:58`, đổi 18/10/2026), chỉ nhận ESP-NOW từ `esp32R` rồi in lên Serial Monitor — cho phép xem log khi robot chạy không dây, không cần kéo dây USB theo robot
- 1 mã ArUco dán trên nóc robot, hướng đầu marker trùng hướng đầu robot

## Cấu trúc thư mục

```
Core/Inc, Core/Src          — toàn bộ code tự viết (driver + logic điều khiển)
Core/Startup                — startup assembly cho STM32F401RETx
esp32_vision/esp32_robot    — firmware esp32R: cầu ESP-NOW <-> UART, relay log (đang dùng)
esp32_vision/esp32_server_log — firmware esp32Server: nhận log qua ESP-NOW, in ra USB (đang dùng)
esp32_vision/pc_vision      — script phát pose giả để test khi chưa có camera
esp32_teleop/               — bản điều khiển bàn phím cũ, giữ làm lịch sử, không còn dùng
matlab_plot/                — script Octave/MATLAB vẽ lại log (fwmr_parse_log.m, compare_enc_vs_imu.m, plot_ekf_runs.m, compare_3_methods.m, fwmr_metrics_xy.m) + log .txt + hình đã xuất
tests_luu_tru/              — mã các bài test đã gỡ khỏi Core (.c.txt/.h.txt) + config_cac_bai_test.h.txt (khối config tương ứng, cách khôi phục)
bao_cao_ekf.pdf             — báo cáo toán EKF (chứng minh từng bước)
so-do-chan-FWMR.pdf         — sơ đồ chân hoàn chỉnh, dựng lại từ chính mã nguồn (06/10/2026)
tan_so_lay_mau_FWMR.xlsx    — bảng tần số lấy mẫu các bài đo
FWMR_run_test.ioc           — cấu hình STM32CubeMX (clock, v.v.) — KHÔNG đồng bộ với code thật, xem dưới
STM32F401RETX_*.ld          — linker script (Flash / RAM)
```

## Kiến trúc firmware

| Module | Vai trò |
|---|---|
| `config.h` | Hằng số cấu hình trung tâm: PWM, hình học robot, hệ số trượt, hiệu chuẩn động cơ, vòng giữ hướng, tham số camera, tham số EKF. Không còn tham số bài test nào |
| `gpio.h/c` | Cấu hình chân GPIO điều khiển chiều quay (DIR) 4 động cơ + chân STBY driver |
| `motor.h/c` | Driver PWM (TIM1, 4 kênh, 20 kHz). `speed_to_duty()` dùng mô hình `duty = OFFSET[bánh] + K_FF·|ω|`; `motor_set_duty_raw()` ghi thẳng duty bỏ qua offset/K_FF, dùng cho bài đo duty chung |
| `encoder.h/c` | Đọc encoder 4 bánh bằng Timer Encoder Mode (TIM2/TIM3/TIM4/TIM5), xử lý tràn 16/32-bit, `ENC_SIGN` riêng từng bánh |
| `kinematics.h/c` | Động học thuận (FK) & nghịch (IK) cho mecanum 4 bánh (pseudo-inverse, bất biến qua các bài chạy) |
| `mpu6050.h/c` | Driver I2C1 mức thanh ghi cho MPU6050, hiệu chuẩn bias + tích phân gyro Z ra góc yaw, cho phép ghi đè yaw từ nguồn tuyệt đối |
| `odometry.h/c` | FK từ encoder → vận tốc thân xe, nhân hệ số trượt `K_SLIP_X/Y`, xoay theo yaw IMU, tích phân ra (x, y) trong vòng lặp chính; cho phép ghi đè (x, y) từ nguồn tuyệt đối |
| `pose_link.h/c` | Parse dòng lệnh từ gateway: pose tuyệt đối (xử lý), `STOP` (xử lý — `pose_link_take_stop()`), `START`/`WPLIST`/`WPCLR` (nhận biết, chưa xử lý); giữ pose mới nhất + watchdog + bộ đếm gói |
| `usart6.h/c` | TX: telemetry 115200 baud. RX: ngắt RXNE gom dòng vào **hàng đợi vòng 8 dòng** (PC7) |
| `ekf.h/c` | **EKF 4 trạng thái** [X, Y, θ, bias gyro] trộn camera (tuyệt đối, trễ, nhiễu) với encoder + gyro (50 Hz, trôi dần) — xem [Bộ lọc EKF](#bộ-lọc-ekf-ekfhc) |
| `strafe_cal_test.h/c`, `rot_cal_test.h/c` | Hiệu chuẩn chạy ngang / quay tại chỗ — bước 0–1 của chương zigzag, không bắt buộc camera |
| `zigzag_test.h/c` | **Bài test hiện đang bật** — 19 bước đi thẳng + quay tại chỗ + quay đầu 180° + về, 3 chế độ `ZZ_MODE` (thuần encoder / encoder+IMU / EKF+camera) |
| `robot_control.h/c` | API cấp cao duy nhất: `robot_set_velocity(vx, vy, wz)` → tự IK + xuất PWM 4 bánh |
| `main.c` | Boot → hiệu chuẩn IMU → (chỗ trống để gọi bài test mới) → vòng lặp 50 Hz: đọc pose + encoder + IMU → odometry → ghi đè pose khi có khung hình mới → vòng P-controller giữ hướng → `robot_set_velocity()` → log telemetry |

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

**Duty PWM (feed-forward + offset bù ma sát tĩnh):** `duty = OFFSET[bánh] + K_FF · |ω|`, mỗi bánh một `OFFSET` riêng (`MOTOR_DUTY_OFFSET_FL/FR/RL/RR`). Bảng offset đo **không tải** (23–24/09/2026) không còn đúng ở duty cao/có tải: bài chạy 2.4 m thuần encoder (06–07/10/2026) cho thấy bánh **RR yếu hẳn** so với 3 bánh còn lại (chậm hơn RL 7–10% ở cùng duty), một đặc điểm cơ khí của riêng cụm RR, không phải do bù sai — xem [Các bài test đã lưu trữ](#các-bài-test-đã-lưu-trữ-tests_luu_tru).

**Vòng hiệu chỉnh hướng (P-controller), dùng trong vòng lặp chính:** yaw đo từ MPU6050 so với `THETA_TARGET_DEG`, sai số nhân `HEADING_KP` (giới hạn ±`HEADING_MAX_WZ`) rồi cộng thẳng vào lệnh `ωz`.

**Odometry (`odometry.c`):** vận tốc thân xe lấy từ FK trên delta-encoder, nhân hệ số trượt `K_SLIP_X`/`K_SLIP_Y` (bù quãng đường thật nhỏ hơn quãng đường bánh lăn — ước lượng tạm 0.97/0.75, chưa đo riêng từng trục), xoay theo yaw IMU, tích phân Euler ra (x, y). Nguồn tuyệt đối từ camera ghi đè trực tiếp qua `odometry_set_pose()`/`mpu6050_set_yaw_deg()` trong vòng lặp chính — cách trộn "thô" này chỉ là mặc định của vòng lặp chính; bài test chiếm toàn quyền tự quản lý pose riêng (EKF hoặc odometry thuần).

## Vòng lặp điều khiển chính

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
| `STOP` | Dừng khẩn | **Đã xử lý** (15/10/2026) — `pose_link_take_stop()` trả 1 lần cờ STOP; bài test tự kiểm tra mỗi chu kỳ |
| `START` | Cho phép chạy | Nhận biết, chưa xử lý |
| `WPLIST;<x1>;<y1>;<x2>;<y2>;…` | Danh sách điểm đích | Nhận biết, chưa xử lý |
| `WPCLR` | Xoá danh sách điểm đích | Nhận biết, chưa xử lý |

**Quy ước đơn vị** (firmware tự quy đổi khi parse): `x`, `y` tính bằng **centimet** đổi sang mét; `theta` tính bằng **radian** đổi sang độ. Chiều dương của θ phải là **ngược chiều kim đồng hồ nhìn từ trên xuống**, trùng chiều dương gyro Z — đã kiểm chứng trên phần cứng 24/09/2026, lệch 0.1° sau khi xoay 90°.

**Không có checksum trên đường truyền.** Gateway gửi text thuần, không magic byte, không CRC. Hai lớp chặn duy nhất: esp32R lọc ký tự không in được, và `pose_link` từ chối giá trị ngoài `POSE_XY_ABS_MAX_CM` hoặc θ ngoài ±7 rad. Các bài test EKF (đã lưu trữ) thêm một lớp nữa ở tầng ứng dụng: nếu marker bị che, PC có thể gửi lại pose **cũ giống hệt** — lúc robot đang di chuyển thì 2 khung giống nhau tuyệt đối là bất thường, nên bị bỏ qua thay vì đưa vào EKF.

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

**Sửa 19/10/2026 — nạp lại có bù trễ:** bản 18/10 nạp lại sau 10 khung bị loại bằng `ekf_set_pose(z)`, tức gán pose camera của ~200 ms trước cho hiện tại và xoá lịch sử → EKF tụt sau xe ~10 cm, vượt đích 111–164 mm. Bản mới cộng độ lệch (z − trạng thái lịch sử tại t_meas) vào trạng thái hiện tại **và** cả lịch sử, đặt lại P của X, Y, θ về R, giữ bias. `ekf_reset_jump()` trả độ nhảy vị trí; `ekf_run_test` in `EKFRUN:NAP_LAI nhay_mm=...` mỗi lần nạp lại. Q, R, trễ, cổng giữ nguyên như 18/10 để so sánh chỉ một thay đổi.

## Bài zigzag (chương hiện tại)

19 bước, hệ xuất phát (m): W0(0;0) → W1(2.4;0) → W2(2.4;0.6) → W3(0;0.6) → W4(0;1.2) → W5(2.4;1.2), quay đầu 180° tại W5, về theo đúng đường cũ. Chỉ đi thẳng + quay tại chỗ (bỏ chạy ngang — không ổn định, xe trôi chéo, thử 20/10). 3 biến thể `ZZ_MODE`: 0 thuần encoder, 1 encoder+IMU (đang bật), 2 EKF+camera. Theo yêu cầu mentor: **không** nhân hệ số trượt (K_SLIP, k_rot) ở bất kỳ bài nào trong chương này.

Kết quả 2 lần/biến thể, 20/10/2026 (chi tiết + log: `matlab_plot/`, doc dự án `dan_y_zigzag_19102026.md`):
- V1 (thuần encoder): trôi hướng tới −81°/−103°, sai số về W0 1.5–1.7 m — không dùng được, chỉ làm cận dưới.
- V2 (encoder+IMU): sai số nhỏ (12–55 mm) nhưng tự chấm theo chính góc IMU đang điều khiển, không phải số thật.
- V3 (EKF+camera): số thật đo bằng camera, sai số về W0 16–45 mm. Khung EKF bị loại 21–23% (so với 7% ở bài chạy thẳng) — nghi `EKF_Q_GYRO_SCALE` (0.02) quá nhỏ so với sai số tỉ lệ lúc quay, đề xuất tăng ~0.06 (chưa thử).

## Các bài test đã lưu trữ (`tests_luu_tru/`)

| Bài | File | Tóm tắt | Log |
|---|---|---|---|
| Chạy thẳng 3 m thuần encoder | `enc_run_test` (`ENC_RUN_IMU_HOLD 0`) | duty chung 400, dừng khi 1 bánh bất kỳ đạt 21008 xung | `matlab_plot/log_enc_run_06102026.txt` |
| Chạy thẳng 3 m encoder + IMU | `enc_run_test` (`ENC_RUN_IMU_HOLD 1`, KP 3) | như trên + giữ hướng bằng gyro | `matlab_plot/log_imu_kp3_07102026.txt` |
| Chạy thẳng 3 m EKF | `ekf_run_test` | giữ hướng theo θ EKF + kéo ngang theo lệch ngang EKF, dừng khi quãng đường dọc theo EKF ≥ 3.0 m | `matlab_plot/log_ekf_18102026.txt` |
| Go-to-point | `goto_test` | xoay về đích → chạy → SETTLE/resnap, đích nhập trong config | không log |

Khôi phục 1 bài: chép `xxx.c.txt`/`xxx.h.txt` về `Core/Src`, `Core/Inc` (bỏ đuôi `.txt`), chép khối tương ứng trong `config_cac_bai_test.h.txt` vào `config.h`, thêm `#include` + lời gọi `xxx_run(imu_ready)` vào chỗ đánh dấu trong `main.c`.

## Số liệu hiệu chuẩn đã chốt

| Đại lượng | Giá trị | Ghi chú |
|---|---|---|
| `COUNTS_PER_REV` | 1496 xung/vòng | theo thông số nhà sản xuất (06/10/2026; trước đó dùng 1500, đo thực tế ~1497–1499, lệch <0.15%) |
| Ngưỡng PWM khởi động (không tải) | FL 90, FR 115, RL 125, RR 115 | 23/09/2026 — **không còn đúng ở duty cao/có tải** |
| `K_FF`, `MOTOR_DUTY_OFFSET_*` | 23.3; FL 21, FR 46, RL 56, RR 46 | `offset = ngưỡng − 69`; vẫn dùng cho vòng điều khiển chính |
| Bánh RR yếu hơn cấu trúc | chậm hơn RL 7–10% ở cùng duty | 06–07/10/2026, đo bằng duty chung — chưa sửa trong firmware |
| `MOTOR_OMEGA_MIN` / `ROBOT_V_MIN` | 2.97 rad/s / 0.102 m/s | cận trên, chưa đo thấp hơn |
| Dấu θ camera so với gyro | cùng chiều, lệch 0.1° sau 90° | 24/09/2026, 0° camera trùng trục X |
| Nhiễu θ camera khi đứng yên | σ = 0.53° (`EKF_R_TH_DEG`) | dùng trực tiếp làm R cho EKF |
| `K_SLIP_X` / `K_SLIP_Y` | 0.97 / 0.75 | ước lượng tạm từ 1 bài chạy chéo (28/09/2026), **chưa đo riêng từng trục** — rủi ro chính khi mất camera |
| Độ trễ camera ước lượng | 200 ms (`EKF_CAM_DELAY_MS`) | ước lượng từ bài goto 28/09/2026, **chưa đo lại cho đúng** |

## Thêm một bài chạy mới

Hạ tầng đã sẵn: sau `odometry_reset()` trong `main.c`, pose, yaw, encoder và kênh camera đều chạy. Hai cách:
- Bài chạy dạng **đặt vận tốc**: thay `target_vx_world` / `target_vy_world` trong vòng lặp chính, giữ nguyên vòng giữ hướng và telemetry.
- Bài chạy **chiếm toàn quyền** (có máy trạng thái, log riêng, xem mẫu trong `tests_luu_tru/`): viết vào file riêng `xxx_test.c/h`, gọi `xxx_run(imu_ready)` ở dòng chú thích `// Bai test moi ...` trong `main.c` (ngay trước `STM32:RUNNING`), bật/tắt bằng 1 cờ `#if` trong `config.h`. Log in qua `usart6_send_*` → esp32R → esp32Server (laptop); giữ khuôn `ENC,...` 100 ms nếu muốn dùng lại `fwmr_parse_log.m`.

## Cách test khi chưa có camera

1. **Chỉ STM32, không cần ESP32 nào:** nối USB-TTL vào PC7, gõ tay `100;200;0.78540#`. Dòng `POSE,...` phải nhảy về `1.000, 2.000, 45.00`. Hoặc chạy `python esp32_vision/pc_vision/pose_sim.py COM<x>` để phát liên tục (`--mode static|circle|step`).
2. **Cả chuỗi không dây:** nạp `esp32_vision/esp32_robot/esp32_robot.ino` cho esp32R, rồi `python esp32_vision/pc_vision/pose_sim.py COM<x> --target gateway` để bắn qua gateway. Kiểm tra `PLINK,...`: `ok` tăng đều, `err` và `drop` đứng yên.

## Build & nạp code

1. Cài [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html).
2. Clone repo, import vào STM32CubeIDE (`File → Open Projects from File System…`).
3. **Đóng tab `config.h`/`main.c`/file test đang sửa trước khi build**, hoặc bật Window → Preferences → General → Workspace → "Refresh using native hooks or polling" — IDE có lỗi ghi buffer cũ đè file mới nếu tab còn mở khi sửa file từ ngoài IDE, đã từng làm mất cả buổi debug sai.
4. Build rồi nạp qua ST-Link.
5. Mở Serial Monitor của `esp32R` (hoặc `esp32Server` nếu chạy không dây) để xem `STM32:BOOT`, `POSE,...`, `PLINK,...` (và dòng log của bài test đang gắn).

## Trạng thái & hướng phát triển tiếp theo

Đã xong: driver PWM/encoder/IMU/UART (TX + RX có hàng đợi), động học IK/FK, vòng giữ hướng bằng gyro, kênh camera qua gateway ESP-NOW (kể cả `STOP`), relay log không dây sang laptop, EKF 4 trạng thái (bù trễ, cổng Mahalanobis), 3 bộ log chạy thẳng 3 m (thuần encoder / encoder + IMU / EKF) + script MATLAB so sánh, go-to-point chạy được. 19/10/2026: dọn firmware về hạ tầng sạch, bài test chuyển vào `tests_luu_tru/`.

Việc tiếp theo (chờ quyết định):
- Đã chạy lại bài EKF 19/10 (`matlab_plot/log_ekf_19102026.txt`, `compare_ekf_truoc_sau_sua.m`): vượt đích 105 → 30 mm, khung bị loại 25 → 7 %. Còn lại: camera đôi khi đóng băng/trễ hơn 200 ms (lần 5 đóng băng từ giây 4.5; lần 1 mất 2 s đầu) → cần số thứ tự khung phía PC, và lọc khung trùng cả lúc đứng yên.
- Chốt dừng an toàn theo encoder (vd. 1.2 × quãng đường đích) cho các bài dùng camera — tránh xe trôi khi server camera treo (lần 6, 9 bài EKF).
- Đo lại trễ camera (log cho thấy ~220 ms) và nhiễu XY camera (`EKF_R_XY_M`).
- Phía PC: chỉ gửi khung mới nhất, thêm số thứ tự khung.
- Chạy đủ 7–8 lần/biến thể zigzag với pin đầy (lần 20/10 pin yếu làm V1 chậm 25–35% so với V2); viết script so sánh 3 biến thể.
- Hành vi "hầm" khi mất camera; `START`/`WPLIST`/`WPCLR`; đo riêng `K_SLIP_X/Y`; vòng PID tốc độ từng bánh (bánh RR yếu); đồng bộ `.ioc`.
