#ifndef GOTO_TEST_H
#define GOTO_TEST_H

#include <stdint.h>

// Bai chay toi 1 diem (GTP_GOAL_X_M; GTP_GOAL_Y_M) trong he the gioi.
// Robot dat tai goc (0;0), dau xe theo truc X. Duong di mong muon la doan
// thang tu diem xuat phat toi dich; dau xe giu GTP_HEADING_DEG suot hanh trinh
// (mecanum tinh tien cheo, khong quay dau).
//
// Dinh dang log (moi GTP_LOG_EVERY_MS):
//   GTP,t_ms,v_cmd,x_odo,y_odo,yaw,cam_ok,x_cam,y_cam,th_cam,cross_odo,cross_cam,dist_odo,dFL,dFR,dRL,dRR
//     x,y,cross,dist: mm   yaw,th: do   cross > 0 = lech sang TRAI duong thang
//     dFL..dRR: so xung tung banh trong 100 ms vua qua (da nhan ENC_SIGN)
//
//   GTP:ENC_FAIL / CAM_VUOT_DICH / CAM_LECH_NGANG - dung khan, in ngay luc xay ra
//   GTPSUM,... - tong ket, in lap lai moi 3 giay (ket_qua, sai so, xung tong)
//
// Goi SAU khi hieu chuan IMU. Ham khong bao gio return.
void goto_test_run(uint8_t imu_ready);

#endif
