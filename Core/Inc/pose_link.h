#ifndef POSE_LINK_H
#define POSE_LINK_H

#include <stdint.h>

// Nguon vi tri TUYET DOI tu camera tran + ArUco:
//   PC (Python) --Serial--> esp32C (gateway) --ESP-NOW--> esp32R (tren robot)
//               --UART 115200--> USART6 RX (PC7) --> file nay
//
// Gateway gui goi dang "<ID>;<payload>#", cat ID roi chuyen tiep nguyen
// <payload># cho dung robot. Robot cua du an nay la ID 29.
// Cac dong payload co the nhan duoc:
//
//   <x_cm>;<y_cm>;<theta_rad>   -> POSE (vi tri tuyet doi, xu ly o day)
//   START / STOP                -> nhan biet, CHUA xu ly o buoc nay
//   WPLIST;<x1>;<y1>;<x2>;...   -> nhan biet, CHUA xu ly o buoc nay
//   WPCLR                       -> nhan biet, CHUA xu ly o buoc nay
//
// DON VI TRUYEN (khac don vi firmware, quy doi ngay khi parse):
//   x, y  : cm     -> doi sang met
//   theta : radian -> doi sang do
// Chieu duong theta phai trung chieu duong gyro Z (nguoc chieu kim dong ho
// khi nhin tu tren xuong). Kiem chung bang tay truoc khi tin so lieu.
//
// Goi tin KHONG co checksum/magic, nen kiem tra vung gia tri hop le
// (POSE_XY_ABS_MAX_CM) la lop bao ve duy nhat chong goi nhieu.

void     pose_link_init(void);
void     pose_link_poll(void);        // goi dau moi chu ky dieu khien

uint8_t  pose_link_take_fresh(void);  // 1 = co khung hinh MOI ke tu lan goi truoc (tu xoa co)
uint8_t  pose_link_valid(void);       // 1 = da tung nhan pose VA con trong POSE_LINK_TIMEOUT_MS
uint32_t pose_link_age_ms(void);      // thoi gian ke tu khung hinh cuoi (ms)

float    pose_link_get_x(void);          // met
float    pose_link_get_y(void);          // met
float    pose_link_get_theta_deg(void);  // do

uint32_t pose_link_get_rx_ok(void);    // so dong pose hop le
uint32_t pose_link_get_rx_other(void); // so dong lenh biet nhung chua xu ly (START/STOP/WP*)
uint32_t pose_link_get_rx_err(void);   // so dong khong hieu duoc / ngoai vung hop le

#endif
