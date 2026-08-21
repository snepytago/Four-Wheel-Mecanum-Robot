#include "motor.h"
#include "config.h"
#include "stm32f4xx.h"
#include <math.h>

static uint32_t speed_to_duty(float speed_rad_s)
{
    uint32_t duty = (uint32_t)(K_FF * fabsf(speed_rad_s) + 0.5f);
    if (duty > PWM_MAX - 1) duty = PWM_MAX - 1;
    return duty;
}

static void pwm_pins_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // PA8-PA11 -> AF mode
    GPIOA->MODER &= ~((3u<<16)|(3u<<18)|(3u<<20)|(3u<<22));
    GPIOA->MODER |=  ((2u<<16)|(2u<<18)|(2u<<20)|(2u<<22));

    // AF1 = TIM1 cho ca 4 chan
    GPIOA->AFR[1] &= ~((0xFu<<0)|(0xFu<<4)|(0xFu<<8)|(0xFu<<12));
    GPIOA->AFR[1] |=  ((1u<<0)|(1u<<4)|(1u<<8)|(1u<<12));
}

static void timer1_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    TIM1->PSC = 0;
    TIM1->ARR = PWM_MAX - 1;

    TIM1->CCMR1 = (6u<<4)  | TIM_CCMR1_OC1PE
                | (6u<<12) | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 = (6u<<4)  | TIM_CCMR2_OC3PE
                | (6u<<12) | TIM_CCMR2_OC4PE;

    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0; TIM1->CCR4 = 0;

    TIM1->BDTR |= TIM_BDTR_MOE;   // bat buoc voi advanced timer, khong co thi chan khong ra xung
    TIM1->EGR   = TIM_EGR_UG;
    TIM1->CR1   = TIM_CR1_ARPE | TIM_CR1_CEN;
}

void motor_init(void)
{
    pwm_pins_init();
    timer1_init();
    GPIOA->BSRR = (1u<<5);   // STBY = PA5, mo driver (MODER da cau hinh o gpio_dir_pins_init)
}

void motor_set_FL(float speed_rad_s)
{
    if (speed_rad_s >= 0) { GPIOB->BSRR = (1u<<0);      GPIOA->BSRR = (1u<<(4+16)); }
    else                  { GPIOB->BSRR = (1u<<16);     GPIOA->BSRR = (1u<<4);      }
    TIM1->CCR1 = speed_to_duty(speed_rad_s);
}

void motor_set_FR(float speed_rad_s)
{
    if (speed_rad_s >= 0) { GPIOC->BSRR = (1u<<0);      GPIOC->BSRR = (1u<<(1+16)); }
    else                  { GPIOC->BSRR = (1u<<16);     GPIOC->BSRR = (1u<<1);      }
    TIM1->CCR2 = speed_to_duty(speed_rad_s);
}

void motor_set_RL(float speed_rad_s)
{
    if (speed_rad_s >= 0) { GPIOC->BSRR = (1u<<(4+16)); GPIOC->BSRR = (1u<<5);      }
    else                  { GPIOC->BSRR = (1u<<4);      GPIOC->BSRR = (1u<<(5+16)); }
    TIM1->CCR3 = speed_to_duty(speed_rad_s);
}

void motor_set_RR(float speed_rad_s)
{
    if (speed_rad_s >= 0) { GPIOC->BSRR = (1u<<2);      GPIOC->BSRR = (1u<<(3+16)); }
    else                  { GPIOC->BSRR = (1u<<(2+16)); GPIOC->BSRR = (1u<<3);      }
    TIM1->CCR4 = speed_to_duty(speed_rad_s);
}

void motor_set_all(float w_FL, float w_FR, float w_RL, float w_RR)
{
    motor_set_FL(w_FL);
    motor_set_FR(w_FR);
    motor_set_RL(w_RL);
    motor_set_RR(w_RR);
}
