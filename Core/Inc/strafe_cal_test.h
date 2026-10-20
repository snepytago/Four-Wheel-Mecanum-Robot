#ifndef STRAFE_CAL_TEST_H
#define STRAFE_CAL_TEST_H

#include <stdint.h>

// Hieu chuan chay ngang (mecanum strafe), buoc 0 cua bai zigzag.
// Cap duty ngang FL = -d, FR = +d, RL = +d, RR = -d (SC_DIR = +1: sang trai),
// dung khi 1 banh BAT KY dat SC_TARGET_COUNTS xung (tri tuyet doi).
// SC_USE_CAM = 0: khong can camera, log 14 cot nhu bai thuan encoder.
// SC_USE_CAM = 1: camera chi ghi log (khong dieu khien), log 19 cot.
//   K_SLIP_Y = quang_duong_ngang_that / banh_lan ngang_mm (in o khoi KET QUA)
//
// Log (STM32 -> esp32R -> ESP-NOW -> esp32 laptop), moi ENC_RUN_LOG_EVERY_MS,
// 14 cot (khong camera) hoac 19 cot (co camera), doc duoc bang fwmr_parse_log.m:
//   ENC,t_ms,duty,FL,FR,RL,RR,x_mm,y_mm,th_deg,yaw_deg,gz_dps,u,xi_mm,yi_mm,xc_mm,yc_mm,thc_deg,cam,s
//   x,y,th : odometry thuan encoder (khong bu truot, goc tu encoder)
//   xi,yi  : odometry dung K_SLIP_X/Y hien tai + goc IMU
//   xc,yc  : camera, he xuat phat ; s = 0

void strafe_cal_test_run(uint8_t imu_ready);

#endif
