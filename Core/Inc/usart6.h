#ifndef USART6_H
#define USART6_H

#include <stdint.h>

#define USART6_RX_LINE_MAXLEN   48

void usart6_init(void);
void usart6_send_char(char c);
void usart6_send_string(const char *str);
void usart6_send_int(int32_t val);
void usart6_send_float(float val, uint8_t decimals);

// --- RX: nhan lenh dieu khien thoi gian thuc qua PC7 (USART6_RX) ---
// Goi usart6_rx_init() 1 lan, SAU usart6_init(), luc khoi dong.
void    usart6_rx_init(void);
uint8_t usart6_rx_line_ready(void);   // 1 = co san 1 dong lenh moi cho doc
void    usart6_rx_get_line(char *out_buf, uint16_t max_len);

#endif
