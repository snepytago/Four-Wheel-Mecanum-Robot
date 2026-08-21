#include "usart6.h"
#include "stm32f4xx.h"

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
