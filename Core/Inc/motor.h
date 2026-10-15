#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

typedef enum {
    MOTOR_FL = 0,
    MOTOR_FR,
    MOTOR_RL,
    MOTOR_RR
} motor_id_t;

void motor_init(void);

void motor_set_FL(float speed_rad_s);
void motor_set_FR(float speed_rad_s);
void motor_set_RL(float speed_rad_s);
void motor_set_RR(float speed_rad_s);

void motor_set_all(float w_FL, float w_FR, float w_RL, float w_RR);

// Ghi thang duty PWM (0..PWM_MAX-1), bo qua OFFSET va K_FF. Dau = chieu (duong = tien).
void motor_set_duty_raw(int32_t d_FL, int32_t d_FR, int32_t d_RL, int32_t d_RR);

#endif
