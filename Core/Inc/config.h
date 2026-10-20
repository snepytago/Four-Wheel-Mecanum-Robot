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
// EKF tron camera + encoder + gyro (ekf.c) - trang thai [X, Y, theta, bias gyro]
// ---------------------------------------------------------------------------
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



// ---------------------------------------------------------------------------
// Bai chay thang 3 m bam duong bang EKF (ekf_run_test.c) - khoi phuc 19/10/2026
// Giu nguyen ten va gia tri nhu lan chay 18/10 de so sanh truc tiep
// ---------------------------------------------------------------------------
#define EKF_RUN_TEST_ENABLE     0

#define ER_DIST_M               3.0f    // dung khi quang duong doc duong thang (EKF) dat muc nay
#define ER_LAT_HOLD             1       // 1 = keo ve duong thang theo lech ngang EKF
#define ER_KY                   1.0f    // duty / mm lech ngang
#define ER_KIY                  0.5f    // duty / (mm*s)
#define ER_S_MAX                60.0f
#define ER_SI_MAX               40.0f

#define ENC_RUN_DUTY            400     // duty chung 4 banh
#define ENC_RUN_DUTY_START      130     // duty luc xuat phat
#define ENC_RUN_RAMP_MS         300u
#define ENC_RUN_START_DELAY_MS  3000u   // dem nguoc, xe dung yen (EKF hoc bias)
#define ENC_RUN_TIMEOUT_MS      12000u
#define ENC_RUN_LOG_EVERY_MS    100u
#define ENC_RUN_REST_MS         300u    // khong co xung moi trong ngan nay -> da dung han
#define ENC_RUN_COAST_MAX_MS    3000u

// Giu huong theo goc EKF: u = KP*e + KI*tich_phan(e) - KD*gyro_z (duty)
#define HH_KP                   3.0f
#define HH_KI                   3.0f
#define HH_KD                   0.3f
#define HH_U_MAX                60.0f
#define HH_I_MAX                30.0f

#define GOTO_INIT_FRAMES        10      // so khung camera lay trung binh lam diem xuat phat
#define GOTO_SETTLE_MS          900u    // cho sau khi dung han
#define GOTO_SETTLE_AVG_MS      300u    // trung binh camera trong ngan cuoi
#define GOTO_GZ_MAX_DPS         250.0f  // |gyro z| lon hon -> coi la gai nhieu

// ---------------------------------------------------------------------------
// Hieu chuan chay ngang (strafe_cal_test.c) - buoc 0 bai zigzag, 20/10/2026
// Dung lai: ENC_RUN_START_DELAY_MS/_TIMEOUT_MS/_LOG_EVERY_MS/_REST_MS/_COAST_MAX_MS,
// HH_KP/KI/KD/U_MAX/I_MAX, GOTO_INIT_FRAMES/_SETTLE_MS/_SETTLE_AVG_MS
// ---------------------------------------------------------------------------
#define STRAFE_CAL_TEST_ENABLE  0

#define SC_DIST_M               3.0f    // quang duong BANH LAN (chua bu truot)
// Dung khi 1 banh BAT KY dat so xung nay (tri tuyet doi): 3.0 m -> 21008 xung
#define SC_TARGET_COUNTS        ((int32_t)(SC_DIST_M / (TWO_PI * WHEEL_R) * COUNTS_PER_REV))
#define SC_DIR                  -1      // +1 = sang trai (+y than xe), -1 = sang phai
#define SC_DUTY                 400     // duty ngang; tang neu banh dung/giat
#define SC_DUTY_START           150
#define SC_RAMP_MS              300u
#define SC_IMU_HOLD             0       // 0 = thuan encoder (IMU chi ghi log), 1 = giu huong bang IMU (HH_*)
#define SC_LOG_GUARD            600     // ngung in khi con it hon so xung nay toi moc
#define SC_USE_CAM              0       // 0 = khong can camera ; 1 = camera ghi log (do K_SLIP_Y)

// ---------------------------------------------------------------------------
// Hieu chuan quay tai cho (rot_cal_test.c) - buoc 1 bai zigzag (di - quay - di), 20/10/2026
// Dung lai: ENC_RUN_START_DELAY_MS/_LOG_EVERY_MS/_REST_MS/_COAST_MAX_MS
// ---------------------------------------------------------------------------
#define ROT_CAL_TEST_ENABLE     0

#define RC_ANGLE_DEG            90.0f   // goc ly thuyet moi lan quay (khong bu truot)
// Dung khi 1 banh BAT KY dat so xung nay: banh lan K_GEOM*goc -> 90 do = 1883 xung, 360 do = 7535
#define RC_TARGET_COUNTS        ((int32_t)(K_GEOM * RC_ANGLE_DEG * 0.017453293f / (TWO_PI * WHEEL_R) * COUNTS_PER_REV))
#define RC_N_TURNS              8       // so lan quay trong 1 lan chay
#define RC_PATTERN              1       // 1 = trai,trai,phai,phai,... (nhu zigzag) ; 0 = moi lan cung RC_DIR
#define RC_DIR                  1       // +1 = trai (CCW), -1 = phai
#define RC_DUTY                 250     // duty quay; gyro +-250 do/s nen khong qua ~450
#define RC_DUTY_START           150
#define RC_RAMP_MS              300u
#define RC_TIMEOUT_MS           6000u
#define RC_PAUSE_MS             1000u   // nghi sau khi dung han, roi moi chot goc IMU
#define RC_LOG_GUARD            300     // ngung in khi con it hon so xung nay toi moc
#define RC_GZ_SAT_DPS           240.0f

// ---------------------------------------------------------------------------
// Bai zigzag (zigzag_test.c) - di thang + quay tai cho: 9 buoc di, quay dau 180, 9 buoc ve (19 buoc), 20/10/2026
// Khong nhan he so truot (K_SLIP, k_rot) theo yeu cau mentor.
// Doan thang dung lai: ENC_RUN_DUTY/_DUTY_START/_RAMP_MS/_TIMEOUT_MS, HH_*, ER_KY/KIY/S_MAX/SI_MAX
// Chung: ENC_RUN_START_DELAY_MS/_LOG_EVERY_MS/_REST_MS/_COAST_MAX_MS, GOTO_INIT_FRAMES/_SETTLE_AVG_MS
// ---------------------------------------------------------------------------
#define ZIGZAG_TEST_ENABLE      1

#define ZZ_MODE                 1       // 0 = thuan encoder, 1 = encoder + IMU, 2 = EKF + camera
#define ZZ_LONG_M               2.4f    // doan dai: 2.4 m = 16806 xung
#define ZZ_SHORT_M              0.6f    // doan ngan: 0.6 m = 4201 xung ; quay 90 do = 1883 xung
#define ZZ_STEP_LAST            19      // chay den het buoc nay: 9 = chi chieu di, 19 = di + quay dau + ve ; 1 = chi doan 2.4 m dau
#define ZZ_V2_TURN_IMU          0       // V2: 0 = quay dung theo so xung (nhu V1) ; 1 = quay dung theo goc IMU
#define ZZ_V2_HOLD_ABS          0       // V2: 0 = di thang giu huong luc bat dau doan ; 1 = giu huong dich tuyet doi
#define ZZ_CAM_LOG              1       // mode 0/1: 1 = ghi log camera neu co (cho toi da ZZ_CAM_WAIT_MS)
#define ZZ_CAM_WAIT_MS          5000u

#define ZZ_TURN_DUTY            250     // nhu bai hieu chuan quay 20/10
#define ZZ_TURN_DUTY_START      150
#define ZZ_TURN_RAMP_MS         300u
#define ZZ_TURN_DUTY_MIN        180     // duty cuoi khi cham dan
#define ZZ_TURN_SLOW_DEG        20.0f   // giam duty tuyen tinh trong ngan goc cuoi; 0 = tat
#define ZZ_TURN_TIMEOUT_MS      6000u
#define ZZ_PAUSE_MS             1000u   // nghi sau moi buoc (camera het tre)

#define ZZ_LOG_GUARD_CNT        600     // ngung in log khi con it hon ngan nay toi moc
#define ZZ_LOG_GUARD_DEG        8.0f
#define ZZ_LOG_GUARD_M          0.08f

#if (EKF_RUN_TEST_ENABLE + STRAFE_CAL_TEST_ENABLE + ROT_CAL_TEST_ENABLE + ZIGZAG_TEST_ENABLE) > 1
#error "Chi bat 1 bai test: EKF_RUN / STRAFE_CAL / ROT_CAL / ZIGZAG"
#endif

#endif
