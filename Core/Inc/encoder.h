#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include "motor.h"   // dung chung enum motor_id_t: FL=0,FR=1,RL=2,RR=3

void    encoder_init(void);
int32_t encoder_get_count(motor_id_t wheel);
void    encoder_reset_all(void);
void    encoder_get_deltas(int32_t dcnt[4]);   // dcnt da nhan ENC_SIGN, index theo motor_id_t

int32_t encoder_get_count_signed(motor_id_t wheel);   // da nhan ENC_SIGN, duong = tien
#endif
