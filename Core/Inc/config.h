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

#endif
