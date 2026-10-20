#ifndef EKF_H
#define EKF_H

#include <stdint.h>

// EKF tron camera (tuyet doi, tre, nhieu) voi encoder + gyro (50 Hz, troi dan).
// Trang thai: X, Y (m, he camera G) ; TH (rad, [-pi, pi]) ; B = bias gyro con du (rad/s)
//
//   ekf_predict  : moi chu ky 20 ms, dau vao la do doi than xe tu encoder va gyro z
//   ekf_update_cam: moi khi co khung camera, t_meas_ms = thoi diem CHUP (da tru do tre)

void    ekf_init(float x, float y, float th, uint32_t t_ms);
// Dat lai X, Y, TH (vd tu trung binh camera luc dung yen), giu nguyen bias
void    ekf_set_pose(float x, float y, float th, uint32_t t_ms);
void    ekf_predict(float dx_b, float dy_b, float gz_rad_s, float dt, uint32_t t_ms);

// 1 = nhan khung, 0 = loai (ngoai cong), 2 = loai lien tiep qua nhieu -> nap lai tu camera
//     (co bu tre: dich trang thai + lich su theo do lech tai t_meas, giu bias)
uint8_t ekf_update_cam(float xc, float yc, float thc, uint32_t t_meas_ms);

float    ekf_x(void);
float    ekf_y(void);
float    ekf_th(void);
float    ekf_bias(void);
float    ekf_sigma_xy(void);   // do lech chuan vi tri (m), lay max cua X, Y
uint32_t ekf_n_ok(void);
uint32_t ekf_n_reject(void);
uint32_t ekf_n_reset(void);
float    ekf_reset_jump(void); // do nhay vi tri lan nap lai gan nhat (m)

#endif
