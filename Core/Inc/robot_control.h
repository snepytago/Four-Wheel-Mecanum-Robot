#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

// Goi 1 lan voi (vx, vy, wz) mong muon -> tu dong IK + xuat PWM ca 4 banh.
// Tu gio ve sau: doi huong di chuyen = doi 3 con so nay, khong dung lai code.
void robot_set_velocity(float vx, float vy, float wz);

#endif
