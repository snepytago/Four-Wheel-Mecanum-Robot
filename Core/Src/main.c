#include "main.h"
#include "config.h"
#include "gpio.h"
#include "motor.h"
#include "encoder.h"
#include "kinematics.h"
#include "odometry.h"
#include "usart6.h"
#include "mpu6050.h"
#include "pose_link.h"
#include "pwm_test.h"
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
    usart6_rx_init();   // RX (PC7) - nhan dong "<x_cm>;<y_cm>;<theta_rad>" tu
                        // gateway esp32C qua esp32R; parse trong pose_link.c
    pose_link_init();

    usart6_send_string("STM32:BOOT\r\n");

#if PWM_TEST_ENABLE
    // Bai do nguong PWM: chiem toan quyen dieu khien dong co, do xong thi treo
    // lai in ket qua. KHONG bao gio chay tiep xuong vong dieu khien ben duoi.
    pwm_test_run();
#endif

#if PWM_TEST2_ENABLE
    // Pha 2: do tren san, camera lam trong tai. Can gateway + camera dang chay.
    pwm_test2_run();
#endif

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

#if POSE_LINK_ENABLE
    uint8_t  pose_snapped_once = 0;   // khung hinh dau tien luon lay theta 100%
    uint32_t log_div = 0;
#endif

    while (1) {
        if (HAL_GetTick() - last_step >= CONTROL_DT_MS) {
            last_step += CONTROL_DT_MS;

            uint32_t now_tick = HAL_GetTick();
            float dt_s = (now_tick - last_dt_tick) / 1000.0f;
            last_dt_tick = now_tick;

#if POSE_LINK_ENABLE
            pose_link_poll();   // vet hang doi dong da ve tu ngat USART6 RX
#endif

            int32_t dcnt[4];
            encoder_get_deltas(dcnt);

            if (imu_ready) {
                mpu6050_update(dt_s);
            }
            odometry_update(dcnt, dt_s);

#if POSE_LINK_ENABLE
            // Co khung hinh MOI -> ghi de pose bang nguon tuyet doi tu camera.
            // Khong ghi de lap lai gia tri cu moi chu ky: giua 2 khung hinh,
            // odometry encoder+gyro van chay tiep de giu nhip 50Hz muot.
            if (pose_link_take_fresh()) {
                odometry_set_pose(pose_link_get_x(), pose_link_get_y());

                // Sai lech goc phai chuan hoa ve [-180,180] TRUOC khi tron:
                // yaw gyro cong don khong bi gioi han, con theta camera nam
                // trong [-180,180] - tru thang se nhay 360 do khi vuot bien.
                float yaw_now = mpu6050_get_yaw_deg();
                float d = pose_link_get_theta_deg() - yaw_now;
                while (d >  180.0f) d -= 360.0f;
                while (d < -180.0f) d += 360.0f;

                float alpha = pose_snapped_once ? POSE_YAW_ALPHA : 1.0f;
                mpu6050_set_yaw_deg(yaw_now + alpha * d);
                pose_snapped_once = 1;
            }
#endif

            if (imu_ready) {
                usart6_send_string("IMU,");
                usart6_send_int(mpu6050_get_accel_x());          usart6_send_char(',');
                usart6_send_int(mpu6050_get_accel_y());          usart6_send_char(',');
                usart6_send_int(mpu6050_get_accel_z());          usart6_send_char(',');
                usart6_send_float(mpu6050_get_gyro_z_dps(), 2);  usart6_send_char(',');
                usart6_send_float(mpu6050_get_yaw_deg(), 2);
                usart6_send_string("\r\n");
            }

            // Van toc tinh tien muc tieu trong he THE GIOI - van bang 0 o buoc
            // nay: robot dung yen, chi tu giu huong. Vong go-to-point (tinh
            // target_vx/vy tu sai so den diem dich) se thay 2 dong nay o buoc sau.
            float target_vx_world = 0.0f;
            float target_vy_world = 0.0f;

            float theta_now_deg = odometry_get_theta_deg();
            float theta_now_rad = theta_now_deg * 0.017453293f;   // deg -> rad

            // World -> body: nghich dao dung phep xoay da dung trong odometry_update (R(theta)^T)
            float vx_body =  target_vx_world * cosf(theta_now_rad) + target_vy_world * sinf(theta_now_rad);
            float vy_body = -target_vx_world * sinf(theta_now_rad) + target_vy_world * cosf(theta_now_rad);

            // Vong giu huong bang gyro (P-controller): sai so so voi THETA_TARGET_DEG,
            // gioi han +-HEADING_MAX_WZ.
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

#if POSE_LINK_ENABLE
            // PLINK,<valid>,<age_ms>,<x_cam>,<y_cam>,<theta_cam>,<ok>,<other>,<err>,<drop>
            //   valid = 0 -> dang mu (mat marker/mat song), age_ms = -1 -> chua tung nhan
            //   other     -> dong START/STOP/WPLIST nhan ra nhung chua xu ly
            //   err       -> dong khong hieu duoc hoac ngoai vung hop le
            //   drop      -> dong bi bo vi hang doi RX day (main loop doc khong kip)
            if (++log_div >= POSE_LOG_EVERY_N) {
                log_div = 0;
                usart6_send_string("PLINK,");
                usart6_send_int(pose_link_valid());                 usart6_send_char(',');
                usart6_send_int((int32_t)pose_link_age_ms());       usart6_send_char(',');
                usart6_send_float(pose_link_get_x(), 3);            usart6_send_char(',');
                usart6_send_float(pose_link_get_y(), 3);            usart6_send_char(',');
                usart6_send_float(pose_link_get_theta_deg(), 2);    usart6_send_char(',');
                usart6_send_int((int32_t)pose_link_get_rx_ok());    usart6_send_char(',');
                usart6_send_int((int32_t)pose_link_get_rx_other()); usart6_send_char(',');
                usart6_send_int((int32_t)pose_link_get_rx_err());   usart6_send_char(',');
                usart6_send_int((int32_t)usart6_rx_get_dropped());
                usart6_send_string("\r\n");
            }
#endif
        }
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
