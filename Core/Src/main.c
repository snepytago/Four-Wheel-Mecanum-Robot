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
#include "robot_control.h"

int main(void)
{
    HAL_Init();

    gpio_dir_pins_init();
    motor_init();
    encoder_init();
    usart6_init();

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

//    robot_set_velocity(0.0f, 0.0f, 0.3f);

    encoder_reset_all();
    odometry_reset();
    usart6_send_string("STM32:RUNNING\r\n");

    uint32_t last_step   = HAL_GetTick();
    uint32_t last_dt_tick = last_step;
    uint32_t start_tick  = last_step;   // moc thoi gian t=0 cho quy dao cung tron (e)

    // --- Trang thai cho test hinh vuong (SQUARE) ---
    uint8_t sq_seg    = 0;      // 0..3: chi so canh dang di (0=+X,1=+Y,2=-X,3=-Y), nguoc kim dong ho
    float   sq_seg_x0 = 0.0f;   // toa do (x,y) tai thoi diem BAT DAU canh hien tai
    float   sq_seg_y0 = 0.0f;
    uint8_t sq_done   = 0;
    const float SQ_DIR_X[4] = {  1.0f, 0.0f, -1.0f,  0.0f };
    const float SQ_DIR_Y[4] = {  0.0f, 1.0f,  0.0f, -1.0f };

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


#if TEST_SQUARE_ENABLE
            // --- Test hinh vuong: 4 doan thang lien tiep, GIU NGUYEN huong, khong quay o goc ---
            float dir_x = SQ_DIR_X[sq_seg];
            float dir_y = SQ_DIR_Y[sq_seg];

            float target_vx_world = sq_done ? 0.0f : SQUARE_SPEED_MPS * dir_x;
            float target_vy_world = sq_done ? 0.0f : SQUARE_SPEED_MPS * dir_y;

            // Quang duong da di DUNG THEO HUONG canh hien tai (chieu vi tri len vector dir),
            // khong dung khoang cach Euclid tho de tranh dung som/tre khi bi lech ngang chut it
            float seg_dx = odometry_get_x() - sq_seg_x0;
            float seg_dy = odometry_get_y() - sq_seg_y0;
            float seg_dist = seg_dx * dir_x + seg_dy * dir_y;

            if (!sq_done && seg_dist >= SQUARE_SIDE_M) {
                sq_seg_x0 = odometry_get_x();
                sq_seg_y0 = odometry_get_y();
                sq_seg++;
                if (sq_seg >= 4) sq_done = 1;
            }
#elif TEST_ARC_ENABLE
            // --- Test (e): quy dao tham so cua cung tron ban kinh r1 quanh tam O ---
            // Chon O sao cho robot xuat phat dung tren duong tron: x(t)=r1*sin(phase), y(t)=r1*(1-cos(phase))
            // => van toc tiep tuyen trong he THE GIOI la dao ham theo t:
            float t_s   = (now_tick - start_tick) / 1000.0f;      // thoi gian tinh tu luc bat dau di (giay)
            float phase = TEST_ARC_OMEGA_RADS * t_s;               // rad da quet quanh O tu luc xuat phat

            float target_vx_world = TEST_ARC_R1_M * TEST_ARC_OMEGA_RADS * cosf(phase);
            float target_vy_world = TEST_ARC_R1_M * TEST_ARC_OMEGA_RADS * sinf(phase);
#else
            float target_vx_world = TARGET_VX_WORLD;
            float target_vy_world = TARGET_VY_WORLD;
#endif

            // main.c — trong control loop, thay khoi robot_set_velocity hien tai
            float theta_now_deg = odometry_get_theta_deg();
            float theta_now_rad = theta_now_deg * 0.017453293f;   // deg -> rad

            // World -> body: nghich dao dung phep xoay da dung trong odometry_update (R(theta)^T)
            float vx_body =  target_vx_world * cosf(theta_now_rad) + target_vy_world * sinf(theta_now_rad);
            float vy_body = -target_vx_world * sinf(theta_now_rad) + target_vy_world * cosf(theta_now_rad);

            // theta_now se ~THETA_TARGET_DEG suot qua trinh (test e KHONG doi huong), vong nay
            // chi co tac dung "ghim" lai neu bi truot/nhieu, dung y het test thang
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

#if TEST_SQUARE_ENABLE
            // Trang thai hinh vuong: canh dang di (0..3) + quang duong da di tren canh do (de overlay Excel)
            usart6_send_string("SQ,");
            usart6_send_int(sq_seg);                usart6_send_char(',');
            usart6_send_float(seg_dist, 3);
            usart6_send_string("\r\n");

            // --- Dieu kien dung: da di het ca 4 canh (sq_done=1, dat trong nhanh tinh o tren) ---
            if (sq_done) break;
#elif TEST_ARC_ENABLE
            // Vi tri THAM CHIEU ly thuyet tren cung tron (de overlay so sanh voi POSE thuc te trong Excel)
            usart6_send_string("ARC_REF,");
            usart6_send_float(TEST_ARC_R1_M * sinf(phase), 3);              usart6_send_char(',');
            usart6_send_float(TEST_ARC_R1_M * (1.0f - cosf(phase)), 3);
            usart6_send_string("\r\n");

            // --- Dieu kien dung: da quet du goc cung mong muon ---
            float arc_done_deg = phase * (180.0f / PI);
            if (arc_done_deg >= TEST_ARC_ANGLE_DEG) break;
#else
            // --- Dieu kien dung: khoang cach Euclidean tu goc, khong phai chi rieng truc x ---
            float dist = sqrtf(odometry_get_x() * odometry_get_x()
                              + odometry_get_y() * odometry_get_y());
            if (dist >= TARGET_DIST_M) break;
#endif
        }
    }

    motor_set_all(0.0f, 0.0f, 0.0f, 0.0f);

    // Xuat tong so xung encoder tich luy tu luc encoder_reset_all() dau lenh chay
    usart6_send_string("ENC,");
    usart6_send_int(encoder_get_count_signed(MOTOR_FL)); usart6_send_char(',');
    usart6_send_int(encoder_get_count_signed(MOTOR_FR)); usart6_send_char(',');
    usart6_send_int(encoder_get_count_signed(MOTOR_RL)); usart6_send_char(',');
    usart6_send_int(encoder_get_count_signed(MOTOR_RR));
    usart6_send_string("\r\n");

    usart6_send_string("STM32:DONE\r\n");
    while (1) { }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
