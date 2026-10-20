#ifndef ENC_RUN_TEST_H
#define ENC_RUN_TEST_H

#include <stdint.h>

// Bai chay thang (ENC_RUN_DIST_M): dung khi 1 banh bat ky dat ENC_RUN_TARGET_COUNTS xung.
// Che do duty chung co the bat them vong giu huong bang IMU (ENC_RUN_IMU_HOLD).
//
// Log qua USART6 -> esp32R -> ESP-NOW -> esp32Server:
//   ENC,t_ms,cmd,FL,FR,RL,RR,x_mm,y_mm,th_deg,yaw_deg,gz_dps,u,xi_mm,yi_mm
//     moi ENC_RUN_LOG_EVERY_MS
//     FL..RR: xung luy ke tu luc xuat phat (da nhan ENC_SIGN, duong = tien)
//     x,y,th: odometry thuan encoder ; yaw,gz: IMU ; u: hieu chinh duty
//     xi,yi : odometry encoder + huong IMU
//   ENCRUN:CHAM_MOC ...                            luc 1 banh cham moc
//   ===== KET QUA ===== ...                        in lai moi 5 s
//
// Goi 1 lan luc boot. Ham khong bao gio return.
void enc_run_test_run(uint8_t imu_ready);

#endif
