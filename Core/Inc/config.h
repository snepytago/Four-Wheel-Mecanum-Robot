#ifndef CONFIG_H
#define CONFIG_H

#define PWM_MAX         800u
#define DUTY(percent)   ((percent) * PWM_MAX / 100u)

#define WHEEL_R  0.034f
#define K_GEOM   0.17125f

// ---------------------------------------------------------------------------
// Quy doi toc do goc banh (rad/s) -> duty PWM.   duty = OFFSET[banh] + K_FF*|w|
// ---------------------------------------------------------------------------
// Dang cu "duty = K_FF*|w|" KHONG co offset la sai ve nguyen tac: dong co co
// ma sat tinh nen can mot luong duty toi thieu moi bat dau quay. Hau qua la o
// toc do thap duty roi xuong duoi nguong va banh dung im, con o toc do cao lai
// cap thua. Hai sai so nguoc chieu nhau nen chinh rieng K_FF khong bao gio sua
// duoc - phai them so hang offset.
//
// So lieu do duoc (4 bai do, 23-24/09/2026 - xem pwm_test.c):
//   nguong khoi dong tung banh, ke banh khong tai:  FL 90, FR 115, RL 125, RR 115
//   duong dac tinh co tai (pha 2 co bu nguong):     v = 0.00146*(duty_tb - 42)
//   -> K_FF = 23.3 ; offset[banh] = nguong[banh] - 69
//   Kiem chung cheo: thay w = 2.97 rad/s vao ra dung duty 90 cho FL (= nguong
//   do duoc) va v = 0.101 m/s, khop voi 0.102 m/s do thuc te.
//
// K_FF cu = 29.3 (va truoc do 66.0) da bi bo: cach hieu chinh cu chi nhan/chia
// ca he so ma khong biet den offset nen khong the dung cho ca dai toc do.
#define K_FF     23.3f

#define MOTOR_DUTY_OFFSET_FL   21.0f
#define MOTOR_DUTY_OFFSET_FR   46.0f
#define MOTOR_DUTY_OFFSET_RL   56.0f
#define MOTOR_DUTY_OFFSET_RR   46.0f

// Toc do goc banh nho nhat ma banh THUC SU quay duoc tren san, va toc do robot
// tuong ung khi ca 4 banh cung o muc do. Duoi muc nay duty roi xuong duoi
// nguong khoi dong: firmware van cap PWM nhung banh khong nhuc nhich.
// => vong go-to-point KHONG duoc ra lenh cham hon ROBOT_V_MIN, neu khong robot
//    se dung im trong khi van "tuong" la dang di toi dich.
// Day la can TREN: bai do bat dau tu dung muc nay nen chua thu thap hon duoc.
#define MOTOR_OMEGA_MIN        2.97f
#define ROBOT_V_MIN            0.102f

#define COUNTS_PER_REV  1500.0f
#define TWO_PI          6.283185307f

#ifndef PI
#define PI              3.14159265359f
#endif

#define CONTROL_DT_MS   20u

// Goc huong than xe muon giu, doc lap voi van toc tinh tien (vx,vy) - vong
// P-controller ben duoi tu chinh wz de bam theo goc nay bang gyro MPU6050.
#define THETA_TARGET_DEG   0.0f

#define HEADING_KP         0.02f
#define HEADING_MAX_WZ     0.5f

// ---------------------------------------------------------------------------
// Nguon vi tri TUYET DOI tu camera tran (gateway esp32C -> esp32R -> USART6 RX)
// ---------------------------------------------------------------------------
// 1 = moi khung hinh camera ghi de (x, y, theta) cua robot; giua 2 khung hinh
//     odometry encoder + gyro van tiep tuc tich phan binh thuong.
// 0 = bo qua hoan toan du lieu camera, chay dead-reckoning thuan nhu truoc.
#define POSE_LINK_ENABLE       1

// Qua thoi gian nay khong nhan duoc dong pose nao -> pose_link_valid() = 0
// (marker bi che, camera treo, mat song ESP-NOW). Robot KHONG tu dong lam gi
// o buoc nay vi chua di chuyen; khi them go-to-point thi day la dieu kien dung.
#define POSE_LINK_TIMEOUT_MS   500u

// Goi tin tu gateway KHONG co checksum: day la lop chan duy nhat chong goi
// nhieu/hong. Dat theo kich thuoc vung camera quet duoc (don vi cm).
#define POSE_XY_ABS_MAX_CM     1000.0f

// He so lay theta tu camera: 1.0 = tin camera hoan toan moi khung hinh.
// Giam xuong (vd 0.2-0.4) neu theta tu ArUco nhieu - khi do gyro giu phan
// ngan han, camera chi keo dan ve gia tri tuyet doi. Rieng khung hinh DAU
// TIEN luon lay 1.0 de co goc goc tuyet doi ngay lap tuc.
#define POSE_YAW_ALPHA         1.0f

// In dong telemetry PLINK moi N chu ky dieu khien (N=10 -> 5Hz). Khong in moi
// chu ky vi USART6 TX dang blocking, 3 dong/20ms se cham tran thoi gian.
#define POSE_LOG_EVERY_N       10u

// ---------------------------------------------------------------------------
// Bai do nguong PWM khoi dong 4 banh (pwm_test.c) - xem pwm_test.h
// ---------------------------------------------------------------------------
// 1 = boot vao thang bai do roi TREO lai, KHONG chay vong dieu khien binh
//     thuong (de vong giu huong bang gyro khong can thiep vao PWM khi dang do).
// NHO DOI VE 0 va nap lai sau khi do xong.
#define PWM_TEST_ENABLE        0

#define PWM_TEST_STEP          5u    // moi nac tang duty bao nhieu
#define PWM_TEST_DWELL_MS      400u  // giu moi nac bao lau truoc khi doc encoder
#define PWM_TEST_MAX_DUTY      400u  // 50% PWM_MAX - den day chua quay thi bo cuoc

// So xung toi thieu trong 1 nac de coi la DA QUAY. 20 xung / 1500 xung moi
// vong ~ 5 do - du de phan biet quay that voi rung tai cho.
#define PWM_TEST_MIN_COUNTS    20

// So nac LIEN TIEP phai quay duoc moi cong nhan la nguong. Mot nac don le co
// the chi la dong co giat mot cai vuot ma sat tinh roi ket lai - doi 3 nac
// lien nhau thi loai duoc truong hop do. Nguong lay la nac DAU cua chuoi.
#define PWM_TEST_CONFIRM_STEPS 3

// Sau khi chot nguong thi chay them bao nhieu nac de lay duong dac tinh
// duty->toc do (do doc doan nay = K_FF thuc cua rieng banh do). Khong quet
// het dai vi banh dang ke tren khong, quay nhanh qua khong can thiet.
#define PWM_TEST_EXTRA_STEPS   10

// --- PHA 2: do tren san that, ca 4 banh cung duty, camera lam trong tai ---
// Chi bat 1 trong 2 pha mot luc. Pha 2 CAN gateway + camera dang chay.
#define PWM_TEST2_ENABLE        0

// Kiem tra quy uoc dau theta camera (pose_theta_check_run): chi doc, khong
// chay dong co. Bat -> boot xong la vao thang, xoay robot bang tay de xem.
// DA CHAY 24/09/2026: camera va gyro CUNG CHIEU, goc 0 do trung truc X,
// hai nguon lech nhau 0.1 do sau khi xoay 90 do -> quy uoc DUNG, giu nguyen.
// Nhieu theta camera luc dung yen: do lech chuan 0.53 do (R cho Kalman).
#define POSE_CHECK_ENABLE       0

// --- Bai chay thang: kiem chung K_FF, vong giu huong, do tre camera ---
// CAN gateway + camera dang chay, va khoang trong >1.5m phia truoc dau xe.
#define STRAIGHT_TEST_ENABLE    1

#define ST_TARGET_V             0.15f   // m/s - tren V_MIN (0.102) mot khoang an toan
#define ST_RAMP_MS              500u    // tang toc dan, tranh truot banh luc khoi hanh
#define ST_HOLD_MS              3000u   // doan giu toc do on dinh - dung de do v_thuc
#define ST_COAST_MS             2000u   // sau khi cat lenh, theo doi tiep de do do tre
#define ST_LOG_EVERY_MS         100u

// Nguong PWM khoi dong tung banh, do duoc o pha 1 (ke banh, khong tai).
// Sua lai theo ket qua PWMSUM cua chinh robot neu do lai lan nua.
#define PWM_TH_FL               90u
#define PWM_TH_FR               115u
#define PWM_TH_RL               125u
#define PWM_TH_RR               115u

// 0 = ca 4 banh cung MOT duty (do tho, thay ro banh nao ket truoc).
// 1 = moi banh duoc cong nguong rieng cua no: duty[w] = PWM_TH[w] + delta.
//     Ca 4 banh cung "xuat phat" tai delta = 0, nen day vua la phep do vua
//     la phep THU cach bu vung chet: neu bu dung thi dtheta phai nho han va
//     4 cot encoder phai deu nhau han so voi che do 0.
//     LUU Y: khi bat, cot dau tien trong log la DELTA, khong phai duty.
#define PWM_TEST2_USE_OFFSET    1

#define PWM_TEST2_STEP          5u

#if PWM_TEST2_USE_OFFSET
  #define PWM_TEST2_START       0u     // delta bat dau tu 0
  #define PWM_TEST2_MAX_DUTY    150u   // delta toi da (duty that = + PWM_TH)
#else
  #define PWM_TEST2_START       PWM_TEST2_STEP
  #define PWM_TEST2_MAX_DUTY    400u
#endif

#define PWM_TEST2_DWELL_MS      1200u  // thoi gian cap duty moi nac
#define PWM_TEST2_SETTLE_MS     1000u  // cho robot dung han + pose bat kip

// Quang duong toi thieu (met) trong 1 nac de coi la robot DA DI CHUYEN.
// Phai lon hon nhieu lan nhieu vi tri cua camera, neu khong se nham nhieu
// thanh chuyen dong. 25mm la an toan khi camera nhieu vai mm.
// Dat theo DWELL: 40mm trong 1.2s tuong duong 0.033 m/s.
#define PWM_TEST2_MIN_MOVE_M    0.040f

#define PWM_TEST2_CONFIRM_STEPS 3
#define PWM_TEST2_EXTRA_STEPS   12

#endif
