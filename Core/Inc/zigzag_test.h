#ifndef ZIGZAG_TEST_H
#define ZIGZAG_TEST_H

#include <stdint.h>

// Bai zigzag: di thang + quay tai cho, 19 buoc (he xuat phat, m):
//   Di : tien 2.4 -> trai 90 -> tien 0.6 -> trai 90 -> tien 2.4 -> phai 90 -> tien 0.6 -> phai 90 -> tien 2.4
//   Quay dau 180 (trai) tai W5
//   Ve : tien 2.4 -> trai 90 -> tien 0.6 -> trai 90 -> tien 2.4 -> phai 90 -> tien 0.6 -> phai 90 -> tien 2.4
//   W0 (0;0) W1 (2.4;0) W2 (2.4;0.6) W3 (0;0.6) W4 (0;1.2) W5 (2.4;1.2), ve theo thu tu W5 -> W0
//   Huong dich tinh tuyet doi, cong don: di 0/90/180/90/0, ve 180/270/360/270/180 do
// Khong nhan he so truot (K_SLIP, k_rot) o bat ky dau.
// ZZ_MODE 0: thuan encoder - doan thang va doan quay dung theo so xung ly thuyet
// ZZ_MODE 1: encoder + IMU - dung theo xung nhu V1 (ca thang lan quay); IMU chi giu huong khi di thang
//            (giu huong luc bat dau doan). ZZ_V2_TURN_IMU / ZZ_V2_HOLD_ABS = 1 de quay theo IMU / giu huong dich
// ZZ_MODE 2: EKF + camera  - doan thang dung theo quang duong EKF + PID huong + PI keo ve duong;
//                            doan quay dung theo goc EKF
//
// Log (STM32 -> esp32R -> ESP-NOW -> esp32 laptop), moi ENC_RUN_LOG_EVERY_MS:
//   ENC,t_ms,duty,FL,FR,RL,RR,x_mm,y_mm,th_deg,yaw_deg,gz_dps,u,xi_mm,yi_mm[,xc_mm,yc_mm,thc_deg,cam,s]
//   x,y,th : odometry thuan encoder
//   yaw,xi,yi : mode 0/1 = goc IMU + quang duong encoder ; mode 2 = EKF
//   xc,yc,thc : camera (he xuat phat), chi in khi co camera ; s = hieu chinh ngang (mode 2)
//   duty : quay trai > 0, quay phai < 0
// Dong su kien: ZZ:BUOC k=.. (bat dau buoc), ZZ:KQ k=.. (ket qua buoc)

void zigzag_test_run(uint8_t imu_ready);

#endif
