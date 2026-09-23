#include "mpu6050.h"
#include "stm32f4xx_hal.h"

#define MPU6050_ADDR         0xD0
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

static int16_t accel_x, accel_y, accel_z;
static int16_t gyro_z_raw;
static float   gyro_z_dps      = 0.0f;
static float   yaw_angle_deg   = 0.0f;
static int32_t gyro_z_bias_raw = 0;

/* ---- I2C1 low-level (PB8=SCL, PB9=SDA) - noi bo, khong xuat ra ngoai ---- */
static void i2c1_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOB->MODER   &= ~((3u << (8*2)) | (3u << (9*2)));
    GPIOB->MODER   |=  ((2u << (8*2)) | (2u << (9*2)));
    GPIOB->OTYPER  |=  (1u << 8) | (1u << 9);
    GPIOB->PUPDR   &= ~((3u << (8*2)) | (3u << (9*2)));
    GPIOB->PUPDR   |=  ((1u << (8*2)) | (1u << (9*2)));
    GPIOB->OSPEEDR |=  ((3u << (8*2)) | (3u << (9*2)));

    GPIOB->AFR[1]  &= ~((0xFu << ((8-8)*4)) | (0xFu << ((9-8)*4)));
    GPIOB->AFR[1]  |=  ((4u   << ((8-8)*4)) | (4u   << ((9-8)*4)));

    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    I2C1->CR2   |= 16;
    I2C1->CCR    = 80;
    I2C1->TRISE  = 17;

    I2C1->CR1 |= I2C_CR1_PE;
}

static uint8_t i2c1_start(void)
{
    uint32_t timeout = 100000;
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)) { if (--timeout == 0) return 0; }
    return 1;
}

static void i2c1_stop(void) { I2C1->CR1 |= I2C_CR1_STOP; }

static uint8_t i2c1_write_byte(uint8_t data)
{
    uint32_t timeout = 100000;
    while (!(I2C1->SR1 & I2C_SR1_TXE)) { if (--timeout == 0) return 0; }
    I2C1->DR = data;
    timeout = 100000;
    while (!(I2C1->SR1 & I2C_SR1_BTF)) { if (--timeout == 0) return 0; }
    return 1;
}

static uint8_t i2c1_address(uint8_t address)
{
    uint32_t timeout = 100000;
    I2C1->DR = address;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) { if (--timeout == 0) return 0; }
    volatile uint32_t tmp;
    tmp = I2C1->SR1; tmp = I2C1->SR2; (void)tmp;
    return 1;
}

static void mpu_write_reg(uint8_t reg, uint8_t value)
{
    if (!i2c1_start()) return;
    if (!i2c1_address(MPU6050_ADDR)) { i2c1_stop(); return; }
    i2c1_write_byte(reg);
    i2c1_write_byte(value);
    i2c1_stop();
}

static void mpu_read_buffer(uint8_t reg, uint8_t *buffer, uint8_t size)
{
    if (!i2c1_start()) return;
    if (!i2c1_address(MPU6050_ADDR)) { i2c1_stop(); return; }
    i2c1_write_byte(reg);

    if (!i2c1_start()) return;
    if (!i2c1_address(MPU6050_ADDR | 0x01)) { i2c1_stop(); return; }

    if (size == 1) {
        I2C1->CR1 &= ~I2C_CR1_ACK;
        i2c1_stop();
        uint32_t timeout = 100000;
        while (!(I2C1->SR1 & I2C_SR1_RXNE)) { if (--timeout == 0) return; }
        buffer[0] = I2C1->DR;
    } else {
        I2C1->CR1 |= I2C_CR1_ACK;
        for (uint8_t i = 0; i < size; i++) {
            if (i == size - 1) {
                I2C1->CR1 &= ~I2C_CR1_ACK;
                i2c1_stop();
            }
            uint32_t timeout = 100000;
            while (!(I2C1->SR1 & I2C_SR1_RXNE)) { if (--timeout == 0) return; }
            buffer[i] = I2C1->DR;
        }
    }
}

static void mpu_read_raw(void)
{
    uint8_t raw[14];
    mpu_read_buffer(MPU6050_ACCEL_XOUT_H, raw, 14);

    accel_x = (int16_t)(raw[0] << 8 | raw[1]);
    accel_y = (int16_t)(raw[2] << 8 | raw[3]);
    accel_z = (int16_t)(raw[4] << 8 | raw[5]);
    gyro_z_raw = (int16_t)(raw[12] << 8 | raw[13]);
}

uint8_t mpu6050_probe(void)
{
    i2c1_init();
    uint8_t who_am_i = 0;
    mpu_read_buffer(MPU6050_WHO_AM_I, &who_am_i, 1);
    return who_am_i;
}

void mpu6050_start(void)
{
    HAL_Delay(100);
    mpu_write_reg(MPU6050_PWR_MGMT_1, 0x00);
    HAL_Delay(10);

    int32_t sum = 0;
    const int N = 500;
    for (int i = 0; i < N; i++) {
        mpu_read_raw();
        sum += gyro_z_raw;
        HAL_Delay(2);
    }
    gyro_z_bias_raw = sum / N;
}

void mpu6050_update(float dt_s)
{
    mpu_read_raw();

    int32_t gz_corrected = gyro_z_raw - gyro_z_bias_raw;
    gyro_z_dps = gz_corrected / 131.0f;

    yaw_angle_deg += gyro_z_dps * dt_s;
}

int16_t mpu6050_get_accel_x(void)    { return accel_x; }
int16_t mpu6050_get_accel_y(void)    { return accel_y; }
int16_t mpu6050_get_accel_z(void)    { return accel_z; }
float   mpu6050_get_gyro_z_dps(void) { return gyro_z_dps; }
float   mpu6050_get_yaw_deg(void)    { return yaw_angle_deg; }

void    mpu6050_set_yaw_deg(float yaw_deg) { yaw_angle_deg = yaw_deg; }
