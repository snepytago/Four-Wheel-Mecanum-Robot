#ifndef ROT_CAL_TEST_H
#define ROT_CAL_TEST_H

#include <stdint.h>

// Hieu chuan quay tai cho, buoc 1 cua bai zigzag (di - quay - di).
// Quay trai (CCW): FL = -d, FR = +d, RL = -d, RR = +d. Quay phai: doi dau.
// Moi lan quay dung khi 1 banh BAT KY dat RC_TARGET_COUNTS xung (tinh tu dau lan quay),
// troi den khi dung han, nghi RC_PAUSE_MS roi quay lan tiep theo. Khong can camera.
// RC_PATTERN = 1: trai, trai, phai, phai, ... (giong 4 lan quay cua zigzag)
//
// Log (STM32 -> esp32R -> ESP-NOW -> esp32 laptop), 14 cot, doc bang fwmr_parse_log.m:
//   ENC,t_ms,duty,FL,FR,RL,RR,x_mm,y_mm,th_deg,yaw_deg,gz_dps,u,xi_mm,yi_mm
//   duty : co dau (+ trai, - phai) ; u = so thu tu lan quay k
//   th   : goc theo encoder (FK, khong bu truot) ; yaw : goc IMU
// Moi lan quay in 1 dong:
//   QUAY:KQ k=.. huong=.. t_cat_ms=.. t_dung_ms=.. enc_cat=.. enc_dung=.. imu_cat=.. imu_dung=..
//          vot=.. k_rot=.. wz_tb=.. gz_max=.. tx_mm=.. ty_mm=..

void rot_cal_test_run(uint8_t imu_ready);

#endif
