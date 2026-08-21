#include "encoder.h"
#include "stm32f4xx.h"

static const int32_t ENC_SIGN[4] = {-1, -1, -1, 1};   // FL,FR,RL,RR - chinh lai neu dao dau

static void encoder_pins_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    // FL: PA0, PA1 -> TIM5, AF2
    GPIOA->MODER &= ~((3u<<0)|(3u<<2));
    GPIOA->MODER |=  ((2u<<0)|(2u<<2));
    GPIOA->PUPDR &= ~((3u<<0)|(3u<<2));
    GPIOA->PUPDR |=  ((1u<<0)|(1u<<2));
    GPIOA->AFR[0] &= ~((0xFu<<0)|(0xFu<<4));
    GPIOA->AFR[0] |=  ((2u<<0)|(2u<<4));

    // FR: PA6, PA7 -> TIM3, AF2
    GPIOA->MODER &= ~((3u<<12)|(3u<<14));
    GPIOA->MODER |=  ((2u<<12)|(2u<<14));
    GPIOA->PUPDR &= ~((3u<<12)|(3u<<14));
    GPIOA->PUPDR |=  ((1u<<12)|(1u<<14));
    GPIOA->AFR[0] &= ~((0xFu<<24)|(0xFu<<28));
    GPIOA->AFR[0] |=  ((2u<<24)|(2u<<28));

    // RL: PA15 -> TIM2 CH1 (AF1), PB3 -> TIM2 CH2 (AF1)
    GPIOA->MODER &= ~(3u<<30);
    GPIOA->MODER |=  (2u<<30);
    GPIOA->PUPDR &= ~(3u<<30);
    GPIOA->PUPDR |=  (1u<<30);
    GPIOA->AFR[1] &= ~(0xFu<<28);
    GPIOA->AFR[1] |=  (1u<<28);

    GPIOB->MODER &= ~(3u<<6);
    GPIOB->MODER |=  (2u<<6);
    GPIOB->PUPDR &= ~(3u<<6);
    GPIOB->PUPDR |=  (1u<<6);
    GPIOB->AFR[0] &= ~(0xFu<<12);
    GPIOB->AFR[0] |=  (1u<<12);

    // RR: PB6, PB7 -> TIM4, AF2
    GPIOB->MODER &= ~((3u<<12)|(3u<<14));
    GPIOB->MODER |=  ((2u<<12)|(2u<<14));
    GPIOB->PUPDR &= ~((3u<<12)|(3u<<14));
    GPIOB->PUPDR |=  ((1u<<12)|(1u<<14));
    GPIOB->AFR[0] &= ~((0xFu<<24)|(0xFu<<28));
    GPIOB->AFR[0] |=  ((2u<<24)|(2u<<28));
}

static void encoder_timers_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN
                   | RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM5EN;

    TIM5->CCMR1 = (1u<<0) | (1u<<8);   // FL - 32-bit
    TIM5->SMCR  = (3u<<0);
    TIM5->ARR   = 0xFFFFFFFF;
    TIM5->CR1   = TIM_CR1_CEN;

    TIM3->CCMR1 = (1u<<0) | (1u<<8);   // FR - 16-bit
    TIM3->SMCR  = (3u<<0);
    TIM3->ARR   = 0xFFFF;
    TIM3->CR1   = TIM_CR1_CEN;

    TIM2->CCMR1 = (1u<<0) | (1u<<8);   // RL - 32-bit
    TIM2->SMCR  = (3u<<0);
    TIM2->ARR   = 0xFFFFFFFF;
    TIM2->CR1   = TIM_CR1_CEN;

    TIM4->CCMR1 = (1u<<0) | (1u<<8);   // RR - 16-bit
    TIM4->SMCR  = (3u<<0);
    TIM4->ARR   = 0xFFFF;
    TIM4->CR1   = TIM_CR1_CEN;
}

void encoder_init(void)
{
    encoder_pins_init();
    encoder_timers_init();
}

int32_t encoder_get_count(motor_id_t wheel)
{
    switch (wheel) {
        case MOTOR_FL: return (int32_t)TIM5->CNT;
        case MOTOR_FR: return (int16_t)TIM3->CNT;   // 16-bit -> ep kieu de giu dau am
        case MOTOR_RL: return (int32_t)TIM2->CNT;
        case MOTOR_RR: return (int16_t)TIM4->CNT;
        default:       return 0;
    }
}
// encoder.c — them ham nay, dat sau encoder_get_count()
int32_t encoder_get_count_signed(motor_id_t wheel)
{
    return ENC_SIGN[wheel] * encoder_get_count(wheel);
}
void encoder_reset_all(void)
{
    TIM5->CNT = 0;
    TIM3->CNT = 0;
    TIM2->CNT = 0;
    TIM4->CNT = 0;
}

void encoder_get_deltas(int32_t dcnt[4])
{
    static uint32_t p5 = 0, p2 = 0;
    static uint16_t p3 = 0, p4 = 0;

    uint32_t n5 = TIM5->CNT;
    dcnt[MOTOR_FL] = ENC_SIGN[MOTOR_FL] * (int32_t)(n5 - p5);
    p5 = n5;

    uint16_t n3 = (uint16_t)TIM3->CNT;
    dcnt[MOTOR_FR] = ENC_SIGN[MOTOR_FR] * (int32_t)(int16_t)(n3 - p3);
    p3 = n3;

    uint32_t n2 = TIM2->CNT;
    dcnt[MOTOR_RL] = ENC_SIGN[MOTOR_RL] * (int32_t)(n2 - p2);
    p2 = n2;

    uint16_t n4 = (uint16_t)TIM4->CNT;
    dcnt[MOTOR_RR] = ENC_SIGN[MOTOR_RR] * (int32_t)(int16_t)(n4 - p4);
    p4 = n4;
}
