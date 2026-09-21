#include "main.h"
#include "config.h"
#include "gpio.h"
#include "motor.h"
#include "encoder.h"
#include "kinematics.h"
#include "odometry.h"
#include "usart6.h"
#include "mpu6050.h"
#include <math.h>
#include <stdio.h>
#include "robot_control.h"

int main(void)
{
    HAL_Init();

    gpio_dir_pins_init();
    motor_init();
    encoder_init();
    usart6_init();
    usart6_rx_init();   // RX (PC7) - san sang nhan lenh tu 1 ESP32 khac sau nay;
                         // hien CHUA duoc doc/parse trong vong dieu khien ben duoi

    usart6_send_string("STM32:BOOT\r\n");

    uint8_t who_am_i = mpu6050_probe();
    usart6_send_string("STM32:WHO_AM_I=");
    usart6_send_int(who_am_i);
    usart6_send_string("\r\n");

    uint8_t imu_ready = (who_am_i == 0x68 || who_am_i == 0x70);

    if (imu_ready) {
        usart6_send_string("STM32:CALIBRATING\r\n");
        mpu6050_start();
        usart6_send_string("STM32:READY\r\n");
    } else {
        usart6_send_string("STM32:MPU_NOT_FOUND\r\n");
    }

    encoder_reset_all();
    odometry_reset();
    usart6_send_string("STM32:RUNNING\r\n");

    uint32_t last_step   = HAL_GetTick();
    uint32_t last_dt_tick = last_step;

    while (1) {
        if (HAL_GetTick() - last_step >= CONTROL_DT_MS) {
            last_step += CONTROL_DT_MS;

            uint32_t now_tick = HAL_GetTick();
            float dt_s = (now_tick - last_dt_tick) / 1000.0f;
            last_dt_tick = now_tick;

            int32_t dcnt[4];
            encoder_get_deltas(dcnt);

            if (imu_ready) {
                mpu6050_update(dt_s);
            }
            odometry_update(dcnt, dt_s);

            if (imu_ready) {
                usart6_send_string("IMU,");
                usart6_send_int(mpu6050_get_accel_x());          usart6_send_char(',');
                usart6_send_int(mpu6050_get_accel_y());          usart6_send_char(',');
                usart6_send_int(mpu6050_get_accel_z());          usart6_send_char(',');
                usart6_send_float(mpu6050_get_gyro_z_dps(), 2);  usart6_send_char(',');
                usart6_send_float(mpu6050_get_yaw_deg(), 2);
                usart6_send_string("\r\n");
            }

            // Van toc tinh tien muc tieu trong he THE GIOI - mac dinh 0 (robot
            // dung yen, chi tu giu huong bang gyro). TODO sau nay: doc tu lenh
            // nhan qua UART6 RX (usart6_rx_line_ready()/usart6_rx_get_line(),
            // gui tu 1 ESP32 khac) thay vi hang so 0.0f co dinh o day.
            float target_vx_world = 0.0f;
            float target_vy_world = 0.0f;

            float theta_now_deg = odometry_get_theta_deg();
            float theta_now_rad = theta_now_deg * 0.017453293f;   // deg -> rad

            // World -> body: nghich dao dung phep xoay da dung trong odometry_update (R(theta)^T)
            float vx_body =  target_vx_world * cosf(theta_now_rad) + target_vy_world * sinf(theta_now_rad);
            float vy_body = -target_vx_world * sinf(theta_now_rad) + target_vy_world * cosf(theta_now_rad);

            // Vong giu huong bang gyro (P-controller): sai so so voi THETA_TARGET_DEG,
            // gioi han +-HEADING_MAX_WZ - dung y het cau hinh dieu khien truoc khi
            // co cac kich ban test rieng (cung tron/hinh vuong).
            float theta_err_deg = THETA_TARGET_DEG - theta_now_deg;
            float wz_cmd = HEADING_KP * theta_err_deg;
            if (wz_cmd >  HEADING_MAX_WZ) wz_cmd =  HEADING_MAX_WZ;
            if (wz_cmd < -HEADING_MAX_WZ) wz_cmd = -HEADING_MAX_WZ;

            robot_set_velocity(vx_body, vy_body, wz_cmd);

            usart6_send_string("POSE,");
            usart6_send_float(odometry_get_x(), 3);          usart6_send_char(',');
            usart6_send_float(odometry_get_y(), 3);          usart6_send_char(',');
            usart6_send_float(odometry_get_theta_deg(), 2);
            usart6_send_string("\r\n");
        }
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
