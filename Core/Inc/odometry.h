#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>

void  odometry_reset(void);                              // dat lai x=0, y=0
void  odometry_update(const int32_t dcnt[4], float dt_s); // goi moi vong lap voi delta-count encoder + dt do thuc te

float odometry_get_x(void);          // met
float odometry_get_y(void);          // met
float odometry_get_theta_deg(void);  // do - lay truc tiep tu IMU yaw, khong tich phan lai

#endif
