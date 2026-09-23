#ifndef CONFIG_H
#define CONFIG_H

#define PWM_MAX         800u
#define DUTY(percent)   ((percent) * PWM_MAX / 100u)

#define WHEEL_R  0.034f
#define K_GEOM   0.17125f
// K_FF hieu chinh tu log test (e) r1=0.3m, omega=0.3rad/s: fit duong tron thuc te ra R_actual=0.673m
// (~2.25x r1 danh dat, khop voi ty le tong quang duong di thuc/ly thuyet = 2.255x)
// => toc do banh thuc te ~2.25x toc do lenh voi K_FF cu => giam K_FF theo dung ty le do:
// K_FF_moi = K_FF_cu / 2.25 = 66.0 / 2.25 ~= 29.3
#define K_FF     29.3f

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
#define PWM_TEST2_ENABLE        1

#define PWM_TEST2_STEP          5u
#define PWM_TEST2_DWELL_MS      700u   // thoi gian cap duty moi nac
#define PWM_TEST2_SETTLE_MS     900u   // cho robot dung han + pose bat kip
#define PWM_TEST2_MAX_DUTY      400u

// Quang duong toi thieu (met) trong 1 nac de coi la robot DA DI CHUYEN.
// Phai lon hon nhieu lan nhieu vi tri cua camera, neu khong se nham nhieu
// thanh chuyen dong. 25mm la an toan khi camera nhieu vai mm.
#define PWM_TEST2_MIN_MOVE_M    0.025f

#define PWM_TEST2_CONFIRM_STEPS 3
#define PWM_TEST2_EXTRA_STEPS   12

#endif
