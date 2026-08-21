#include "kinematics.h"
#include "config.h"

/*
 * Ma tran gia nghich dao (pseudo-inverse) M cua he mecanum 4 banh.
 * He co 4 banh (dieu khien duoc) nhung chi 3 bac tu do (vx, vy, wz)
 * -> he du dieu khien (overactuated) -> dung M+ (pseudo-inverse cua
 * ma tran dong hoc thuan J) de tinh nguoc toc do 4 banh tu (vx,vy,wz).
 *
 * [w_FL]        [ 1  -1  -K ]   [vx]
 * [w_FR]  = 1/R [ 1   1   K ] * [vy]
 * [w_RL]        [ 1   1  -K ]   [wz]
 * [w_RR]        [ 1  -1   K ]
 *
 * K = K_GEOM = h+g (hinh hoc robot), R = WHEEL_R (ban kinh banh)
 * Doi hinh hoc robot -> chi sua duy nhat bang IK_MATRIX nay, khong dung
 * lai tung dong cong thuc rieng le.
 */
static const float IK_MATRIX[4][3] = {
    { 1.0f, -1.0f, -K_GEOM },   // FL
    { 1.0f,  1.0f,  K_GEOM },   // FR
    { 1.0f,  1.0f, -K_GEOM },   // RL
    { 1.0f, -1.0f,  K_GEOM },   // RR
};

void inverse_kinematics(float vx, float vy, float wz,
                         float *w_FL, float *w_FR, float *w_RL, float *w_RR)
{
    float v[3] = { vx, vy, wz };
    float *w_out[4] = { w_FL, w_FR, w_RL, w_RR };

    for (int i = 0; i < 4; i++) {
        float sum = 0.0f;
        for (int j = 0; j < 3; j++) {
            sum += IK_MATRIX[i][j] * v[j];
        }
        *w_out[i] = sum / WHEEL_R;
    }
}

void forward_kinematics(float w_FL, float w_FR, float w_RL, float w_RR,
                         float *vx, float *vy, float *wz)
{
    *vx = (WHEEL_R / 4.0f) * ( w_FL + w_FR + w_RL + w_RR);
    *vy = (WHEEL_R / 4.0f) * (-w_FL + w_FR + w_RL - w_RR);
    *wz = (WHEEL_R / (4.0f * K_GEOM)) * (-w_FL + w_FR - w_RL + w_RR);
}
