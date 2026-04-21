#include "motor86.h"

#define MOTOR86_DIR_PORT GPIOA
#define MOTOR86_DIR_PIN  GPIO_Pin_3

#define MOTOR86_EN_PORT  GPIOA
#define MOTOR86_EN_PIN   GPIO_Pin_4

#define MOTOR86_TIM      TIM2
#define MOTOR86_TIM_CH   3
#define MOTOR86_TIM_CLK_HZ 1000000U
#define MOTOR86_RPM_LIMIT 3000

static uint16_t g_ppr = MOTOR86_PPR_DEFAULT;
static uint8_t g_enabled = 0;
static int16_t g_rpm = 0;

static void motor86_apply_enable_pin(uint8_t enable)
{
    if (enable) {
        if (MOTOR86_EN_ACTIVE_LEVEL) {
            GPIO_SetBits(MOTOR86_EN_PORT, MOTOR86_EN_PIN);
        } else {
            GPIO_ResetBits(MOTOR86_EN_PORT, MOTOR86_EN_PIN);
        }
    } else {
        if (MOTOR86_EN_ACTIVE_LEVEL) {
            GPIO_ResetBits(MOTOR86_EN_PORT, MOTOR86_EN_PIN);
        } else {
            GPIO_SetBits(MOTOR86_EN_PORT, MOTOR86_EN_PIN);
        }
    }
}

static void motor86_set_compare(uint16_t compare)
{
#if MOTOR86_TIM_CH == 3
    TIM_SetCompare3(MOTOR86_TIM, compare);
#else
#error "MOTOR86_TIM_CH must be 3 for PA2/TIM2_CH3"
#endif
}

static void motor86_stop_pwm(void)
{
    motor86_set_compare(0);
}

void Motor86_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim_base;
    TIM_OCInitTypeDef tim_oc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, DISABLE);
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, DISABLE);

    gpio.GPIO_Pin = GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = MOTOR86_DIR_PIN | MOTOR86_EN_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    GPIO_SetBits(MOTOR86_DIR_PORT, MOTOR86_DIR_PIN);

    tim_base.TIM_Prescaler = 72U - 1U;
    tim_base.TIM_CounterMode = TIM_CounterMode_Up;
    tim_base.TIM_Period = 1000U - 1U;
    tim_base.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_base.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(MOTOR86_TIM, &tim_base);

    tim_oc.TIM_OCMode = TIM_OCMode_PWM1;
    tim_oc.TIM_OutputState = TIM_OutputState_Enable;
    tim_oc.TIM_Pulse = 0;
    tim_oc.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC3Init(MOTOR86_TIM, &tim_oc);
    TIM_OC3PreloadConfig(MOTOR86_TIM, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(MOTOR86_TIM, ENABLE);
    TIM_Cmd(MOTOR86_TIM, ENABLE);

    Motor86_Enable(1);
    Motor86_SetRpm(0);
}

void Motor86_Enable(uint8_t enable)
{
    g_enabled = enable ? 1U : 0U;
    motor86_apply_enable_pin(g_enabled);

    if (!g_enabled) {
        motor86_stop_pwm();
        g_rpm = 0;
    }
}

void Motor86_SetPpr(uint16_t ppr)
{
    if (ppr == 0U) {
        return;
    }
    g_ppr = ppr;

    if (g_rpm != 0) {
        Motor86_SetRpm(g_rpm);
    }
}

uint16_t Motor86_GetPpr(void)
{
    return g_ppr;
}

void Motor86_SetRpm(int16_t rpm)
{
    uint32_t pulse_hz;
    uint32_t arr;
    uint32_t abs_rpm;

    if (rpm > MOTOR86_RPM_LIMIT) rpm = MOTOR86_RPM_LIMIT;
    if (rpm < -MOTOR86_RPM_LIMIT) rpm = -MOTOR86_RPM_LIMIT;

    if (!g_enabled || rpm == 0) {
        motor86_stop_pwm();
        g_rpm = 0;
        return;
    }

    if (rpm > 0) {
        GPIO_SetBits(MOTOR86_DIR_PORT, MOTOR86_DIR_PIN);
        abs_rpm = (uint32_t)rpm;
    } else {
        GPIO_ResetBits(MOTOR86_DIR_PORT, MOTOR86_DIR_PIN);
        abs_rpm = (uint32_t)(-rpm);
    }

    pulse_hz = (abs_rpm * (uint32_t)g_ppr) / 60U;
    if (pulse_hz < 1U) pulse_hz = 1U;
    if (pulse_hz > (MOTOR86_TIM_CLK_HZ / 2U)) pulse_hz = MOTOR86_TIM_CLK_HZ / 2U;

    arr = MOTOR86_TIM_CLK_HZ / pulse_hz;
    if (arr < 2U) arr = 2U;
    arr -= 1U;

    TIM_SetAutoreload(MOTOR86_TIM, (uint16_t)arr);
    motor86_set_compare((uint16_t)((arr + 1U) / 2U));
    TIM_GenerateEvent(MOTOR86_TIM, TIM_EventSource_Update);

    g_rpm = rpm;
}

int16_t Motor86_GetRpm(void)
{
    return g_rpm;
}
