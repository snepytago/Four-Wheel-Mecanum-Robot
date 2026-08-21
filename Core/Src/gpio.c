#include "gpio.h"
#include "stm32f4xx.h"

void gpio_dir_pins_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    // FL direction: PB0, PA4
    GPIOB->MODER &= ~(3u<<0);
    GPIOB->MODER |=  (1u<<0);
    GPIOA->MODER &= ~(3u<<8);
    GPIOA->MODER |=  (1u<<8);

    // FR direction: PC0, PC1
    GPIOC->MODER &= ~(3u<<0);
    GPIOC->MODER |=  (1u<<0);
    GPIOC->MODER &= ~(3u<<2);
    GPIOC->MODER |=  (1u<<2);

    // RR direction: PC2, PC3
    GPIOC->MODER &= ~(3u<<4);
    GPIOC->MODER |=  (1u<<4);
    GPIOC->MODER &= ~(3u<<6);
    GPIOC->MODER |=  (1u<<6);

    // RL direction: PC4, PC5
    GPIOC->MODER &= ~(3u<<8);
    GPIOC->MODER |=  (1u<<8);
    GPIOC->MODER &= ~(3u<<10);
    GPIOC->MODER |=  (1u<<10);

    // STBY: PA5
    GPIOA->MODER &= ~(3u<<10);
    GPIOA->MODER |=  (1u<<10);
}
