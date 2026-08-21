#include "robot_control.h"
#include "kinematics.h"
#include "motor.h"

void robot_set_velocity(float vx, float vy, float wz)
{
    float w_FL, w_FR, w_RL, w_RR;
    inverse_kinematics(vx, vy, wz, &w_FL, &w_FR, &w_RL, &w_RR);
    motor_set_all(w_FL, w_FR, w_RL, w_RR);
}
