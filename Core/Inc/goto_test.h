#ifndef GOTO_TEST_H
#define GOTO_TEST_H

#include <stdint.h>

// Bai go-to-point: robot dat ngau nhien trong vung camera, tu chay toi dich
// (GOTO_GOAL_X_CM; GOTO_GOAL_Y_CM) tren he camera G.
//
// Vi tri dung de dieu khien = EKF tron camera (tuyet doi, tre) voi encoder + gyro.
// GOTO_FACE_GOAL = 1: xoay tai cho cho mui xe huong ve dich -> chay thang toi ->
// tai dich mui xe van huong theo duong di (S -> G).
//
// Trinh tu: hieu chuan gyro (main.c, xe dung yen) -> cho camera, lay trung binh
// GOTO_INIT_FRAMES khung lam diem xuat phat -> dem nguoc -> chay.
// Gui "29;STOP#" qua gateway de dung khan.
//
// In ra (qua esp32R -> esp32Server):
//   GOTO:...  su kien (doi trang thai)
//   GT,t_ms,trang_thai,x_mm,y_mm,th_deg,xc_mm,yc_mm,thc_deg,dist_mm,loi_huong_deg,v
//       moi GOTO_PRINT_EVERY_MS ; x,y,th = EKF ; xc,yc,thc = khung camera cuoi
//   ===== GOTO KET QUA ===== ...  in lai moi 5 s
//
// Ham khong bao gio return.
void goto_test_run(uint8_t imu_ready);

#endif
