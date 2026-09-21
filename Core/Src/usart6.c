#include "usart6.h"
#include "stm32f4xx.h"
#include <string.h>

void usart6_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART6EN;

    GPIOC->MODER &= ~(3u<<12);
    GPIOC->MODER |=  (2u<<12);

    GPIOC->PUPDR &= ~(3u<<12);
    GPIOC->PUPDR |=  (1u<<12);   // pull-up PC6 chong ky tu rac luc boot

    GPIOC->AFR[0] = (GPIOC->AFR[0] & ~(0xFu << 24)) | (0x8u << 24);   // AF8 = USART6_TX

    USART6->BRR = (8u << 4) | 11u;   // 115200 baud @ PCLK2=16MHz (thay cho 9600)
    USART6->CR1 |= USART_CR1_TE;
    USART6->CR1 |= USART_CR1_UE;
}

void usart6_send_char(char c)
{
    while (!(USART6->SR & (1u << 7)));
    USART6->DR = c;
}

void usart6_send_string(const char *str)
{
    while (*str) usart6_send_char(*str++);
}

void usart6_send_int(int32_t val)
{
    char buf[12]; int i = 0; uint8_t neg = 0;
    if (val < 0) { neg = 1; val = -val; }
    if (val == 0) buf[i++] = '0';
    while (val > 0) { buf[i++] = '0' + (val % 10); val /= 10; }
    if (neg) buf[i++] = '-';
    while (i > 0) usart6_send_char(buf[--i]);
}

void usart6_send_float(float val, uint8_t decimals)
{
    if (val < 0) { usart6_send_char('-'); val = -val; }
    int32_t int_part = (int32_t)val;
    usart6_send_int(int_part);
    usart6_send_char('.');
    float frac = val - (float)int_part;
    for (uint8_t i = 0; i < decimals; i++) {
        frac *= 10.0f;
        int digit = (int)frac;
        usart6_send_char('0' + digit);
        frac -= (float)digit;
    }
}

// ============================================================================
// RX — nhan lenh dieu khien tu 1 ESP32 khac (ha tang, main.c chua tich hop).
//
// Chan PC7 = USART6_RX (AF8), dung chung baudrate/BRR da cau hinh trong
// usart6_init() o tren (KHONG dung lai duoc neu chua goi usart6_init()
// truoc, vi ham nay khong tu bat lai UE/BRR).
//
// Co che: nhan tung byte bang ngat RXNE (khong dung DMA/polling trong
// main loop) -> gom vao buffer "dang xay" (rx_building) -> gap '\n' hoac
// '\r' thi sao chep sang buffer "san sang" (rx_line_ready_buf) va bat co
// cho main loop biet co lenh moi.
// ============================================================================

static volatile char     rx_line_ready_buf[USART6_RX_LINE_MAXLEN];
static volatile uint16_t rx_line_ready_len = 0;
static volatile uint8_t  rx_line_ready_flag = 0;

static volatile char     rx_building[USART6_RX_LINE_MAXLEN];
static volatile uint16_t rx_building_len = 0;

void usart6_rx_init(void)
{
    // Clock GPIOC + USART6 da duoc bat trong usart6_init() (TX) - ham nay
    // PHAI duoc goi SAU usart6_init().

    // PC7 = Alternate Function AF8 (USART6_RX)
    GPIOC->MODER &= ~(3u << 14);
    GPIOC->MODER |=  (2u << 14);        // 10: Alternate function mode

    GPIOC->PUPDR &= ~(3u << 14);
    GPIOC->PUPDR |=  (1u << 14);        // pull-up: tranh doc nhieu khi chua noi day ESP32

    GPIOC->AFR[0] = (GPIOC->AFR[0] & ~(0xFu << 28)) | (0x8u << 28);   // AF8 = USART6_RX tren PC7

    USART6->CR1 |= USART_CR1_RE | USART_CR1_RXNEIE;

    NVIC_SetPriority(USART6_IRQn, 5);
    NVIC_EnableIRQ(USART6_IRQn);
}

uint8_t usart6_rx_line_ready(void)
{
    return rx_line_ready_flag;
}

void usart6_rx_get_line(char *out_buf, uint16_t max_len)
{
    NVIC_DisableIRQ(USART6_IRQn);

    uint16_t n = rx_line_ready_len;
    if (n > (uint16_t)(max_len - 1)) n = (uint16_t)(max_len - 1);
    memcpy(out_buf, (const void *)rx_line_ready_buf, n);
    out_buf[n] = '\0';
    rx_line_ready_flag = 0;

    NVIC_EnableIRQ(USART6_IRQn);
}

void USART6_IRQHandler(void)
{
    if (USART6->SR & USART_SR_RXNE) {
        char c = (char)(USART6->DR & 0xFF);   // doc DR cung tu xoa co RXNE

        if (c == '\n' || c == '\r') {
            if (rx_building_len > 0 && !rx_line_ready_flag) {
                memcpy((void *)rx_line_ready_buf, (const void *)rx_building, rx_building_len);
                rx_line_ready_len = rx_building_len;
                rx_line_ready_flag = 1;
            }
            rx_building_len = 0;
        } else if (rx_building_len < (USART6_RX_LINE_MAXLEN - 1)) {
            rx_building[rx_building_len++] = c;
        } else {
            rx_building_len = 0;   // dong qua dai/mat dong bo -> huy, doi ky tu xuong dong tiep
        }
    }

    // Overrun error (ORE): PHAI doc SR roi doc DR de xoa co, neu khong ngat
    // se bi "ket" va khong nhan them byte nao nua.
    if (USART6->SR & USART_SR_ORE) {
        (void)USART6->DR;
    }
}
