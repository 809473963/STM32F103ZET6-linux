#include "motor.h"
#include "delay.h"

#define MOTOR1_DIR_PORT GPIOA
#define MOTOR1_DIR_PIN  GPIO_Pin_7

#define MOTOR1_PUL_PORT GPIOA
#define MOTOR1_PUL_PIN  GPIO_Pin_8

#define MOTOR1_EN_PORT  GPIOA
#define MOTOR1_EN_PIN   GPIO_Pin_2

static volatile uint8_t motor1_run = 0;
static volatile uint16_t motor1_pulse_us = 250;

static void Motor1_PulseHigh(void)
{
    GPIO_SetBits(MOTOR1_PUL_PORT, MOTOR1_PUL_PIN);
}

static void Motor1_PulseLow(void)
{
    GPIO_ResetBits(MOTOR1_PUL_PORT, MOTOR1_PUL_PIN);
}

static void Motor1_Enable(uint8_t enable)
{
    if (enable) {
        if (MOTOR1_EN_ACTIVE_LEVEL) {
            GPIO_SetBits(MOTOR1_EN_PORT, MOTOR1_EN_PIN);
        } else {
            GPIO_ResetBits(MOTOR1_EN_PORT, MOTOR1_EN_PIN);
        }
    } else {
        if (MOTOR1_EN_ACTIVE_LEVEL) {
            GPIO_ResetBits(MOTOR1_EN_PORT, MOTOR1_EN_PIN);
        } else {
            GPIO_SetBits(MOTOR1_EN_PORT, MOTOR1_EN_PIN);
        }
    }
}

void Motor_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;

    gpio.GPIO_Pin = MOTOR1_PUL_PIN;
    GPIO_Init(MOTOR1_PUL_PORT, &gpio);

    gpio.GPIO_Pin = MOTOR1_DIR_PIN;
    GPIO_Init(MOTOR1_DIR_PORT, &gpio);

    gpio.GPIO_Pin = MOTOR1_EN_PIN;
    GPIO_Init(MOTOR1_EN_PORT, &gpio);

    Motor1_PulseLow();
    GPIO_SetBits(MOTOR1_DIR_PORT, MOTOR1_DIR_PIN);

    // 上电即使能
    Motor1_Enable(1);
    motor1_run = 0;
    motor1_pulse_us = 250;
}

void Motor_Enable(uint8_t motor_id, uint8_t enable)
{
    if (motor_id == MOTOR_1 || motor_id == 0xFF) {
        Motor1_Enable(enable ? 1 : 0);
    }
}

void Motor_SetSpeed(uint8_t motor_id, float speed, uint8_t direction)
{
    uint16_t pulse_us;
    float t;

    if (motor_id != MOTOR_1) {
        return;
    }

    if (speed > 100.0f) speed = 100.0f;
    if (speed < 0.0f) speed = 0.0f;

    if (direction == DIR_STOP || speed <= 0.0f) {
        motor1_run = 0;
        Motor1_PulseLow();
        Motor1_Enable(1);
        return;
    }

    if (direction == DIR_FORWARD) {
        GPIO_SetBits(MOTOR1_DIR_PORT, MOTOR1_DIR_PIN);
    } else {
        GPIO_ResetBits(MOTOR1_DIR_PORT, MOTOR1_DIR_PIN);
    }

    // speed 0~100 -> pulse_us MAX~MIN（越小越快）
    t = speed / 100.0f;
    pulse_us = (uint16_t)(MOTOR1_PULSE_US_MAX - (MOTOR1_PULSE_US_MAX - MOTOR1_PULSE_US_MIN) * t);
    if (pulse_us < MOTOR1_PULSE_US_MIN) pulse_us = MOTOR1_PULSE_US_MIN;
    if (pulse_us > MOTOR1_PULSE_US_MAX) pulse_us = MOTOR1_PULSE_US_MAX;

    motor1_pulse_us = pulse_us;
    Motor1_Enable(1);
    motor1_run = 1;
}

void Motor_Service(void)
{
    uint16_t high_us;
    uint16_t low_us;

    if (!motor1_run) {
        return;
    }

    high_us = motor1_pulse_us / 2;
    low_us = motor1_pulse_us - high_us;
    if (high_us < 2U) high_us = 2U;
    if (low_us < 2U) low_us = 2U;

    Motor1_PulseHigh();
    delay_us(high_us);
    Motor1_PulseLow();
    delay_us(low_us);
}

void Motor_StopAll(void)
{
    Motor_SetSpeed(MOTOR_1, 0.0f, DIR_STOP);
}
