#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

uint8_t mpu6050_probe(void);        // init I2C1 + doc WHO_AM_I (0x68 hoac 0x70 la hop le)
void    mpu6050_start(void);        // reset PWR_MGMT_1 + hieu chuan gyro Z (~1s, robot PHAI dung yen)
void    mpu6050_update(float dt_s); // doc mau moi + tich luy goc yaw, goi dinh ky

int16_t mpu6050_get_accel_x(void);
int16_t mpu6050_get_accel_y(void);
int16_t mpu6050_get_accel_z(void);
float   mpu6050_get_gyro_z_dps(void);
float   mpu6050_get_yaw_deg(void);

// Ghi de goc yaw bang nguon TUYET DOI ben ngoai (theta do tu ArUco).
// Gyro van tiep tuc tich phan tu gia tri moi nay cho toi lan ghi de ke tiep.
void    mpu6050_set_yaw_deg(float yaw_deg);

#endif
