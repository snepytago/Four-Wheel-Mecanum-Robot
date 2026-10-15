#ifndef CONFIG_H
#define CONFIG_H

// ---------------------------------------------------------------------------
// PWM - TIM1, 20 kHz, duty 0..PWM_MAX-1
// ---------------------------------------------------------------------------
#define PWM_MAX         800u
#define DUTY(percent)   ((percent) * PWM_MAX / 100u)

// ---------------------------------------------------------------------------
// Hinh hoc robot
// ---------------------------------------------------------------------------
#define WHEEL_R  0.034f
#define K_GEOM   0.17125f   // g + h

// Ty le quang duong THAT / quang duong encoder dem, theo tung truc than xe.
// Chi dung trong odometry.c. Uoc luong tam, chua do rieng tung truc.
#define K_SLIP_X 0.97f
#define K_SLIP_Y 0.75f

// ---------------------------------------------------------------------------
// Quy doi toc do goc banh (rad/s) -> duty PWM:  duty = OFFSET[banh] + K_FF*|w|
// OFFSET bu ma sat tinh, rieng tung banh (hieu chuan 23-24/09/2026).
// ---------------------------------------------------------------------------
#define K_FF     23.3f

#define MOTOR_DUTY_OFFSET_FL   21.0f
#define MOTOR_DUTY_OFFSET_FR   46.0f
#define MOTOR_DUTY_OFFSET_RL   56.0f
#define MOTOR_DUTY_OFFSET_RR   46.0f

// Toc do goc banh nho nhat ma banh thuc su quay tren san, va toc do robot
// tuong ung khi di thang. Duoi muc nay banh dung im du van co PWM.
// Day la can tren (chua do thap hon).
#define MOTOR_OMEGA_MIN        2.97f
#define ROBOT_V_MIN            0.102f

// ---------------------------------------------------------------------------
// Encoder
// ---------------------------------------------------------------------------
#define COUNTS_PER_REV  1496.0f   // xung/vong truc banh (x4), theo thong so nha san xuat
#define TWO_PI          6.283185307f

#ifndef PI
#define PI              3.14159265359f
#endif

// ---------------------------------------------------------------------------
// Vong dieu khien
// ---------------------------------------------------------------------------
#define CONTROL_DT_MS   20u   // 50 Hz

// Vong giu huong: P-controller tren yaw gyro, doc lap voi (vx, vy)
#define THETA_TARGET_DEG   0.0f
#define HEADING_KP         0.02f
#define HEADING_MAX_WZ     0.5f   // rad/s

// ---------------------------------------------------------------------------
// Pose tuyet doi tu camera tran (gateway esp32C -> esp32R -> USART6 RX)
// ---------------------------------------------------------------------------
// 1 = moi khung hinh moi ghi de (x, y, theta); giua 2 khung hinh odometry
//     encoder + gyro van tich phan binh thuong.
// 0 = bo qua du lieu camera, chay dead-reckoning thuan.
#define POSE_LINK_ENABLE       1

// Qua thoi gian nay khong nhan duoc dong pose nao -> pose_link_valid() = 0
#define POSE_LINK_TIMEOUT_MS   500u

// Goi tin khong co checksum: gioi han vung hop le (cm) la lop chan duy nhat
#define POSE_XY_ABS_MAX_CM     1000.0f

// He so tron theta camera vao yaw: 1.0 = tin camera hoan toan.
// Khung hinh dau tien luon lay 1.0.
#define POSE_YAW_ALPHA         1.0f

// In PLINK moi N chu ky (10 -> 5 Hz). USART6 TX blocking nen khong in moi chu ky.
#define POSE_LOG_EVERY_N       10u

// ---------------------------------------------------------------------------
// Bai chay thang thuan encoder (enc_run_test.c)
// ---------------------------------------------------------------------------
// 1 = boot xong vao thang bai chay roi treo lai in ket qua, khong xuong vong
//     dieu khien chinh. Khong dung IMU, khong dung camera.
#define ENC_RUN_TEST_ENABLE     0

#define ENC_RUN_DIST_M         3.0f
// Dung khi 1 banh BAT KY dat so xung nay (3.00 m, 1496 xung/vong -> 21008)
#define ENC_RUN_TARGET_COUNTS   ((int32_t)(ENC_RUN_DIST_M / (TWO_PI * WHEEL_R) * COUNTS_PER_REV))

#define ENC_RUN_V               0.40f   // m/s
#define ENC_RUN_RAMP_MS         300u    // tang toc tu ROBOT_V_MIN len ENC_RUN_V; 0 = khong ramp
#define ENC_RUN_HEADING_HOLD    0       // 1 = giu huong bang theta tinh tu encoder

// 1 = cap CUNG 1 duty cho ca 4 banh, bo qua OFFSET va K_FF (va ENC_RUN_V,
//     ENC_RUN_HEADING_HOLD) - de xem do lech tu nhien giua 4 banh.
// 0 = chay theo van toc ENC_RUN_V nhu binh thuong.
#define ENC_RUN_EQUAL_DUTY      1
#define ENC_RUN_DUTY            400     // ~0.4 m/s: trung binh duty 4 banh lan chay truoc
#define ENC_RUN_DUTY_START      130     // duty luc xuat phat, tren nguong cao nhat (RL 125)

// --- Giu huong bang IMU (chi dung voi ENC_RUN_EQUAL_DUTY = 1) ---
//   e = 0 - yaw_imu (do)      u = KP*e + KI*tich_phan(e) - KD*gyro_z   (duty)
//   FL, RL = duty - u ;  FR, RR = duty + u      (u > 0 -> xe quay trai)
// Thu tu chinh: (1) chi KP -> kiem tra dau  (2) them KD = 0.3
//               (3) them KI = 3.0 de keo yaw ve dung 0 (mo phong: ~3 s)
#define ENC_RUN_IMU_HOLD        1       // TAM TAT: chay thuan encoder (IMU van doc va ghi log)
#define HH_KP                   3.0f    // duty / do
#define HH_KI                   3.0f    // duty / (do*s)
#define HH_KD                   0.3f    // duty / (do/s)
#define HH_U_MAX                60.0f   // gioi han hieu chinh (duty)
#define HH_I_MAX                30.0f   // gioi han phan tich phan (duty)
#define ENC_RUN_START_DELAY_MS  3000u   // dem nguoc truoc khi chay
#define ENC_RUN_TIMEOUT_MS      12000u

#define ENC_RUN_LOG_EVERY_MS    100u
// USART6 TX blocking (~7 ms/dong): ngung in khi con it hon so xung nay toi moc
// de viec kiem tra dieu kien dung khong bi tre.
#define ENC_RUN_LOG_GUARD       600

#define ENC_RUN_REST_MS         300u    // khong co xung moi trong ngan nay -> xe da dung han
#define ENC_RUN_COAST_MAX_MS    3000u

// ---------------------------------------------------------------------------
// Bai go-to-point: camera + encoder + IMU tron bang EKF (goto_test.c, ekf.c)
// ---------------------------------------------------------------------------
// 1 = boot xong (hieu chuan gyro, xe PHAI dung yen) vao thang bai go-to-point.
#define GOTO_TEST_ENABLE        1

// Toa do dich tren he camera G (cm) - cung don vi voi camera gui len
#define GOTO_GOAL_X_CM          180.0f
#define GOTO_GOAL_Y_CM          60.0f

// 1 = xoay tai cho cho mui xe huong ve dich, chay thang toi, toi noi mui xe
//     huong theo duong di (xuat phat -> dich)
// 0 = giu huong ban dau, chay cheo toi dich
#define GOTO_FACE_GOAL          1

#define GOTO_POS_TOL_M          0.03f   // sai so vi tri chap nhan (kiem tra luc dung yen)
#define GOTO_STOP_M             0.015f  // dang chay: con cach dich chung nay thi cat lenh
#define GOTO_OVERSHOOT_M        0.03f   // xa dan hon diem gan nhat chung nay -> dung, tinh lai
#define GOTO_TH_TOL_DEG         3.0f    // sai so huong chap nhan
#define GOTO_TH_GO_DEG          10.0f   // lech hon muc nay thi xoay tai cho truoc khi chay

#define GOTO_V_MAX              0.25f   // m/s
#define GOTO_ACCEL              0.4f    // m/s^2
#define GOTO_KP_POS             1.0f    // (m/s) / m
#define GOTO_WHEEL_MARGIN       1.2f    // banh cham nhat >= 1.2 x MOTOR_OMEGA_MIN
#define GOTO_KP_TH_DRIVE        2.0f    // (rad/s) / rad - giu huong khi dang chay
#define GOTO_WZ_MAX_DRIVE       0.6f    // rad/s
#define GOTO_KP_TH_ROT          2.0f    // xoay tai cho
#define GOTO_WZ_MIN             0.9f    // rad/s - xoay tai cho can >= 0.6 de banh vuot nguong
#define GOTO_WZ_MAX             1.5f
#define GOTO_ROTATE_MAX_MS      8000u

// Pha dung yen: cho xe het troi + camera het tre, roi lay trung binh camera trong
// GOTO_SETTLE_AVG_MS cuoi. Dung voi do tre camera <= SETTLE - AVG (600 ms).
#define GOTO_SETTLE_MS          900u
#define GOTO_SETTLE_AVG_MS      300u
#define GOTO_RESNAP_M           0.01f   // EKF lech trung binh camera (dung yen) hon muc nay -> dat lai theo camera
#define GOTO_RESNAP_DEG         2.0f
#define GOTO_MAX_TRIES          10      // so lan xoay/chay lai toi da
#define GOTO_INIT_FRAMES        10      // so khung camera lay trung binh lam diem xuat phat
#define GOTO_START_DELAY_MS     3000u
#define GOTO_CAM_LOST_MS        1000u   // dang chay ma mat camera qua lau -> dung, cho camera
#define GOTO_TIMEOUT_MS         60000u
#define GOTO_GZ_MAX_DPS         250.0f  // |gyro z| lon hon -> coi la gai nhieu, giu mau truoc
#define GOTO_PRINT_EVERY_MS     200u    // dong GT,... ; 0 = khong in

// --- EKF: trang thai [X, Y, theta, bias gyro] ---
#define EKF_CAM_DELAY_MS        200u    // do tre camera UOC LUONG (bai goto 28/09), chua do lai
#define EKF_R_XY_M              0.01f   // nhieu x, y camera (do lech chuan) - chua do
#define EKF_R_TH_DEG            0.53f   // nhieu theta camera, do 24/09/2026
#define EKF_Q_ENC_X             0.03f   // sai so quang duong doc than xe (truot 0-3%)
#define EKF_Q_ENC_Y             0.10f   // ngang than xe (K_SLIP_Y chua do ky)
#define EKF_Q_ENC_MIN_M         0.0005f // moi chu ky
#define EKF_Q_GYRO_DPS          0.5f    // nhieu gyro z
#define EKF_Q_GYRO_SCALE        0.02f   // sai so ti le gyro khi xoay
#define EKF_Q_BIAS_DPS          0.02f   // bias troi (do/s/sqrt(s))
#define EKF_B0_DPS              0.5f    // do bat dinh bias luc dau
#define EKF_GATE                16.3f   // chi^2 3 bac tu do, 99.9%
#define EKF_REJECT_RESET        10      // loai lien tiep chung nay khung -> nap lai tu camera

#if ENC_RUN_TEST_ENABLE && GOTO_TEST_ENABLE
#error "Chi bat 1 bai test: ENC_RUN_TEST_ENABLE hoac GOTO_TEST_ENABLE"
#endif

#endif
