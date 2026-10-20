#ifndef EKF_RUN_TEST_H
#define EKF_RUN_TEST_H

#include <stdint.h>

// Bai chay thang ER_DIST_M (3 m) theo huong mui xe luc xuat phat, bam duong
// thang bang pose EKF (camera + encoder + gyro).
//
// Cung khung voi bai enc_run_test de so sanh truc tiep: duty chung ENC_RUN_DUTY,
// ramp ENC_RUN_DUTY_START -> ENC_RUN_DUTY trong ENC_RUN_RAMP_MS. Hieu chinh:
//   u (giu huong, HH_KP/KI/KD tren goc EKF):  FL,RL -u ; FR,RR +u
//   s (keo ve duong thang, ER_KY/ER_KIY tren lech ngang EKF, chay ngang mecanum):
//                                             FL,RR -s ; FR,RL +s   (s > 0 -> sang trai)
// Dung khi quang duong doc duong thang (EKF) dat ER_DIST_M.
//
// Moi toa do in ra deu trong HE XUAT PHAT: goc = diem xuat phat, truc x = huong
// mui xe luc xuat phat (giong cac log enc_run truoc -> MATLAB dung lai duoc).
//
// Log (STM32 -> esp32R -> ESP-NOW -> esp32 laptop), moi ENC_RUN_LOG_EVERY_MS:
//   ENC,t_ms,duty,FL,FR,RL,RR,x_mm,y_mm,th_deg,yaw_deg,gz_dps,u,xi_mm,yi_mm,xc_mm,yc_mm,thc_deg,cam,s
//     x,y,th   : odometry thuan encoder (nhu log cu)
//     yaw      : goc EKF ; xi,yi : vi tri EKF (dung de dieu khien)
//     xc,yc,thc: khung camera cuoi (tre ~EKF_CAM_DELAY_MS) ; cam = so khung tron tu dong truoc
//     u, s     : hieu chinh huong / ngang (duty)
//   ===== KET QUA ===== ...  in lai moi 5 s
//
// Ham khong bao gio return.
void ekf_run_test_run(uint8_t imu_ready);

#endif
