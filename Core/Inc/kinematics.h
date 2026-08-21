#ifndef KINEMATICS_H
#define KINEMATICS_H

void inverse_kinematics(float vx, float vy, float wz,
                         float *w_FL, float *w_FR, float *w_RL, float *w_RR);

void forward_kinematics(float w_FL, float w_FR, float w_RL, float w_RR,
                         float *vx, float *vy, float *wz);

#endif
