#include "odometry.h"
#include "config.h"
#include "kinematics.h"
#include "motor.h"      // enum motor_id_t: FL=0,FR=1,RL=2,RR=3 - de index dcnt[]
#include "mpu6050.h"
#include <math.h>

static float pos_x = 0.0f;
static float pos_y = 0.0f;

void odometry_reset(void)
{
    pos_x = 0.0f;
    pos_y = 0.0f;
}

void odometry_update(const int32_t dcnt[4], float dt_s)
{
    if (dt_s <= 0.0f) return;   // tranh chia 0 hoac buoc thoi gian bat thuong

    // Buoc 1: delta-count encoder -> toc do goc tung banh (rad/s)
    float w_FL = (dcnt[MOTOR_FL] / COUNTS_PER_REV) * TWO_PI / dt_s;
    float w_FR = (dcnt[MOTOR_FR] / COUNTS_PER_REV) * TWO_PI / dt_s;
    float w_RL = (dcnt[MOTOR_RL] / COUNTS_PER_REV) * TWO_PI / dt_s;
    float w_RR = (dcnt[MOTOR_RR] / COUNTS_PER_REV) * TWO_PI / dt_s;

    // Buoc 2: FK -> van toc than xe (body frame). Chi lay vx,vy - wz dung tu gyro (chinh xac hon FK)
    float vx_body, vy_body, wz_unused;
    forward_kinematics(w_FL, w_FR, w_RL, w_RR, &vx_body, &vy_body, &wz_unused);

    // Buoc 3: goc theta hien tai - lay thang tu IMU, KHONG tich phan lai o day
    float theta_rad = mpu6050_get_yaw_deg() * PI / 180.0f;

    // Buoc 4: xoay van toc than xe sang he toa do the gioi (ma tran xoay 2D)
    float vx_world = vx_body * cosf(theta_rad) - vy_body * sinf(theta_rad);
    float vy_world = vx_body * sinf(theta_rad) + vy_body * cosf(theta_rad);

    // Buoc 5: tich phan Euler -> vi tri
    pos_x += vx_world * dt_s;
    pos_y += vy_world * dt_s;
}

float odometry_get_x(void)         { return pos_x; }
float odometry_get_y(void)         { return pos_y; }
float odometry_get_theta_deg(void) { return mpu6050_get_yaw_deg(); }
