#ifndef USART6_H
#define USART6_H

#include <stdint.h>

// Do dai toi da 1 dong nhan duoc. Gateway co the ban goi WPLIST dai toi
// ~240 byte; o buoc hien tai esp32R chi chuyen tiep pose (ngan) nhung de
// san 128 de khong phai sua lai khi them waypoint.
#define USART6_RX_LINE_MAXLEN   128

// So dong giu duoc cung luc. Cac dong tu gateway ve theo cum (vd nhieu
// waypoint lien tiep trong vai ms) trong khi main loop chi doc 50 lan/giay
// -> buffer 1 dong se lam mat gan het. 8 dong la du rong cho nhip do.
#define USART6_RX_QUEUE_LEN     8

void usart6_init(void);
void usart6_send_char(char c);
void usart6_send_string(const char *str);
void usart6_send_int(int32_t val);
void usart6_send_float(float val, uint8_t decimals);

// --- RX: nhan lenh tu gateway qua esp32R, chan PC7 (USART6_RX) ---
// Goi usart6_rx_init() 1 lan, SAU usart6_init(), luc khoi dong.
// Ky tu ket thuc 1 dong: '\n', '\r' HOAC '#' - gateway ket thuc moi goi
// bang '#' va khong gui ky tu xuong dong, nen '#' phai duoc coi la het dong
// (nho vay co the cam thang gateway vao PC7 de test, bo qua esp32R).
void     usart6_rx_init(void);
uint8_t  usart6_rx_line_ready(void);   // 1 = con it nhat 1 dong chua doc
void     usart6_rx_get_line(char *out_buf, uint16_t max_len);
uint32_t usart6_rx_get_dropped(void);  // so dong bi bo vi hang doi day

#endif
