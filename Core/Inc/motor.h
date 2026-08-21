#ifndef MOTOR_H
#define MOTOR_H

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

#endif
