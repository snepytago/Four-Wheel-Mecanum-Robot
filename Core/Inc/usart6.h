#ifndef USART6_H
#define USART6_H

#include <stdint.h>

void usart6_init(void);
void usart6_send_char(char c);
void usart6_send_string(const char *str);
void usart6_send_int(int32_t val);
void usart6_send_float(float val, uint8_t decimals);

#endif
