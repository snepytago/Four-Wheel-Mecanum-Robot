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

#define TARGET_DIST_M   2.4f
#define CONTROL_DT_MS   20u

// config.h — thay TARGET_VX/TARGET_VY bang cap nay, tranh nhap nhang 2 he quy chieu
#define TARGET_VX_WORLD    0.2f    // m/s, theo truc X the gioi
#define TARGET_VY_WORLD    0.0f    // m/s, theo truc Y the gioi
#define THETA_TARGET_DEG   0.0f    // goc huong than xe muon giu, doc lap voi huong di chuyen

#define HEADING_KP         0.02f
#define HEADING_MAX_WZ     0.5f

// === Test (e): di theo cung tron ban kinh r1, GIU NGUYEN huong than xe (khong quay mat) ===
// TAM THOI TAT (0) de chuyen sang test hinh vuong ben duoi. Giu lai cac tham so de bat lai sau.
#define TEST_ARC_ENABLE      0
#define TEST_ARC_R1_M        0.3f     // m - ban kinh r1
#define TEST_ARC_OMEGA_RADS  0.7f     // rad/s - toc do dai target ~0.21 m/s (r1*omega)
#define TEST_ARC_ANGLE_DEG   360.0f   // do dai cung muon di; 360 = di het 1 vong tron kin

// === Test hinh vuong: chay theo 4 canh SQUARE_SIDE_M x SQUARE_SIDE_M, lien tiep ===
// Giong tinh than test (e): GIU NGUYEN huong than xe suot qua trinh, chuyen huong o goc = doi
// vector tinh tien (vx,vy) - KHONG can quay than xe 90 do o moi goc, khai thac dung kha nang
// holonomic cua mecanum (day la diem mecanum lam duoc ma robot vi sai/Ackermann khong the).
#define TEST_SQUARE_ENABLE   1
#define SQUARE_SIDE_M         1.2f    // m - do dai 1 canh hinh vuong
#define SQUARE_SPEED_MPS      0.4f    // m/s - toc do tren moi canh (vung da tung kiem chung on dinh
                                       // voi K_FF=66 cu; neu dang dung K_FF=29.3 moi ma van yeu/giat,
                                       // day cung la du lieu them de danh gia K_FF chu khong rieng test (e))
#endif
