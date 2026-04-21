#include "encoder_motor.h"
#include "delay.h"

#define ENCODER_MOTOR_COUNT 3U
#define PWM_ARR_COUNTS 1000U
#define PWM_FIXED_HZ 1000U
#define ENCODER_MOTOR_LEGACY_ID ENCODER_MOTOR_ID_2

// BTS7960 (IBT-2): PB8=TIM4_CH3 -> L_PWM, PB9=TIM4_CH4 -> R_PWM.
#define BTS7960_TIMER TIM4

#ifndef HAL_Delay
#define HAL_Delay(ms) delay_ms(ms)
#endif
typedef struct {
    TIM_TypeDef *pwm_timer;
    uint8_t pwm_channel;
    GPIO_TypeDef *pwm_port;
    uint16_t pwm_pin;

    TIM_TypeDef *encoder_timer;
    GPIO_TypeDef *encoder_port;
    uint16_t encoder_a_pin;
    uint16_t encoder_b_pin;
} EncoderMotorHw_t;

typedef struct {
    int16_t output_speed;
    uint8_t enabled;
    uint8_t dir_forward;
} EncoderMotorState_t;

static const EncoderMotorHw_t g_hw[ENCODER_MOTOR_COUNT] = {
    {
        TIM1, 1U, GPIOA, GPIO_Pin_8,
        TIM3, GPIOA, GPIO_Pin_6, GPIO_Pin_7,
    },
    {
        TIM2, 2U, GPIOA, GPIO_Pin_1,
        TIM5, GPIOA, GPIO_Pin_0, GPIO_Pin_3,
    },
    {
        TIM2, 3U, GPIOA, GPIO_Pin_2,
        TIM8, GPIOC, GPIO_Pin_6, GPIO_Pin_7,
    },
};

static EncoderMotorState_t g_state[ENCODER_MOTOR_COUNT];

static void force_pb8_afpp_for_tim4_ch3(void)
{
    GPIO_InitTypeDef gpio;

    gpio.GPIO_Pin = GPIO_Pin_8;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
}

static void force_pb9_afpp_for_tim4_ch4(void)
{
    GPIO_InitTypeDef gpio;

    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
}

static void ensure_bts7960_channels_ready(void)
{
    force_pb8_afpp_for_tim4_ch3();
    force_pb9_afpp_for_tim4_ch4();
    TIM_CCxCmd(BTS7960_TIMER, TIM_Channel_3, TIM_CCx_Enable);
    TIM_CCxCmd(BTS7960_TIMER, TIM_Channel_4, TIM_CCx_Enable);
}

static int8_t id_to_index(uint8_t motor_id)
{
    if (motor_id >= ENCODER_MOTOR_ID_1 && motor_id <= ENCODER_MOTOR_ID_3) {
        return (int8_t)(motor_id - ENCODER_MOTOR_ID_1);
    }
    return -1;
}

static void apply_stop(uint8_t idx)
{
    (void)idx;
    // Safety red line: both BTS7960 legs must be zero.
    TIM_SetCompare3(BTS7960_TIMER, 0U); // L_PWM: PB8 / TIM4_CH3
    TIM_SetCompare4(BTS7960_TIMER, 0U); // R_PWM: PB9 / TIM4_CH4
}

void Set_Motor_Speed(int speed)
{
    ensure_bts7960_channels_ready();

    // Clamp to valid range: [-1000, 1000].
    if (speed > (int)PWM_ARR_COUNTS) {
        speed = (int)PWM_ARR_COUNTS;
    } else if (speed < -(int)PWM_ARR_COUNTS) {
        speed = -(int)PWM_ARR_COUNTS;
    }

    if (speed > 0) {
        // Forward: R_PWM locked to 0, L_PWM outputs speed.
        TIM_SetCompare3(BTS7960_TIMER, (uint16_t)speed);
        TIM_SetCompare4(BTS7960_TIMER, 0U);
    } else if (speed < 0) {
        // Reverse: L_PWM locked to 0, R_PWM outputs abs(speed).
        TIM_SetCompare3(BTS7960_TIMER, 0U);
        TIM_SetCompare4(BTS7960_TIMER, (uint16_t)(-speed));
    } else {
        // Dynamic brake/coast command in this project: both PWM = 0.
        TIM_SetCompare3(BTS7960_TIMER, 0U);
        TIM_SetCompare4(BTS7960_TIMER, 0U);
    }
}

void BTS7960_SoftStart_Test(void)
{
    int s;

    for (s = 0; s <= 500; s += 10) {
        Set_Motor_Speed(s);
        HAL_Delay(10);
    }

    HAL_Delay(2000);

    for (s = 500; s >= 0; s -= 10) {
        Set_Motor_Speed(s);
        HAL_Delay(10);
    }

    Set_Motor_Speed(0);
}

static void refresh_pwm_timer_state(void)
{
    uint8_t tim1_needed = g_state[0].enabled;
    uint8_t tim2_needed = (uint8_t)(g_state[1].enabled || g_state[2].enabled);
    uint8_t tim4_needed = g_state[1].enabled;

    TIM_Cmd(TIM1, tim1_needed ? ENABLE : DISABLE);
    TIM_Cmd(TIM2, tim2_needed ? ENABLE : DISABLE);
    TIM_Cmd(TIM4, tim4_needed ? ENABLE : DISABLE);
}

static void init_gpio(void)
{
    GPIO_InitTypeDef gpio;

    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;

    gpio.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_Init(GPIOB, &gpio);

    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_3;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOC, &gpio);
}

static void init_pwm_timers(void)
{
    TIM_TimeBaseInitTypeDef tim_base;
    TIM_OCInitTypeDef tim_oc;

    tim_base.TIM_Prescaler = 72U - 1U;
    tim_base.TIM_CounterMode = TIM_CounterMode_Up;
    tim_base.TIM_Period = PWM_ARR_COUNTS - 1U;
    tim_base.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_base.TIM_RepetitionCounter = 0;

    tim_oc.TIM_OCMode = TIM_OCMode_PWM1;
    tim_oc.TIM_OutputState = TIM_OutputState_Enable;
    tim_oc.TIM_OCPolarity = TIM_OCPolarity_High;
    tim_oc.TIM_Pulse = 0;

    TIM_TimeBaseInit(TIM1, &tim_base);
    TIM_OC1Init(TIM1, &tim_oc);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_SetCompare1(TIM1, 0U);
    TIM_Cmd(TIM1, DISABLE);

    TIM_TimeBaseInit(TIM2, &tim_base);
    TIM_OC2Init(TIM2, &tim_oc);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC3Init(TIM2, &tim_oc);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_SetCompare2(TIM2, 0U);
    TIM_SetCompare3(TIM2, 0U);
    TIM_Cmd(TIM2, DISABLE);

    TIM_TimeBaseInit(TIM4, &tim_base);
    TIM_OC3Init(TIM4, &tim_oc);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC4Init(TIM4, &tim_oc);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_SetCompare3(TIM4, 0U);
    TIM_SetCompare4(TIM4, 0U);
    TIM_Cmd(TIM4, DISABLE);
}

static void init_encoder_timer(TIM_TypeDef *tim)
{
    TIM_TimeBaseInitTypeDef tim_base;

    tim_base.TIM_Prescaler = 0;
    tim_base.TIM_CounterMode = TIM_CounterMode_Up;
    tim_base.TIM_Period = 0xFFFFU;
    tim_base.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_base.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(tim, &tim_base);
    TIM_EncoderInterfaceConfig(tim, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_SetCounter(tim, 0U);
    TIM_Cmd(tim, ENABLE);
}

void EncoderMotor_InitAll(void)
{
    uint8_t i;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC |
                           RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO |
                           RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8,
                           ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 |
                           RCC_APB1Periph_TIM4 |
                           RCC_APB1Periph_TIM5,
                           ENABLE);

    init_gpio();
    // Ensure TIM2 keeps default mapping for legacy channels.
    GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, DISABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, DISABLE);
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, DISABLE);

    // TIM4 default maps CH3/CH4 to PB8/PB9.
    GPIO_PinRemapConfig(GPIO_Remap_TIM4, DISABLE);

    ensure_bts7960_channels_ready();
    init_pwm_timers();

    init_encoder_timer(TIM3);
    init_encoder_timer(TIM5);
    init_encoder_timer(TIM8);

    for (i = 0; i < ENCODER_MOTOR_COUNT; i++) {
        g_state[i].enabled = 0U;
        g_state[i].output_speed = 0;
        g_state[i].dir_forward = 1U;
        apply_stop(i);
    }

    refresh_pwm_timer_state();
}

void EncoderMotor_EnableById(uint8_t motor_id, uint8_t en)
{
    int8_t idx;

    if (motor_id == ENCODER_MOTOR_ID_ALL) {
        EncoderMotor_EnableById(ENCODER_MOTOR_ID_1, en);
        EncoderMotor_EnableById(ENCODER_MOTOR_ID_2, en);
        EncoderMotor_EnableById(ENCODER_MOTOR_ID_3, en);
        return;
    }

    idx = id_to_index(motor_id);
    if (idx < 0) {
        return;
    }

    g_state[(uint8_t)idx].enabled = en ? 1U : 0U;
    if ((uint8_t)idx == 1U) {
        // Re-assert PWM pin modes for motor2 path (PB8/PB9).
        ensure_bts7960_channels_ready();
    }
    if (!g_state[(uint8_t)idx].enabled) {
        g_state[(uint8_t)idx].output_speed = 0;
        g_state[(uint8_t)idx].dir_forward = 1U;
        apply_stop((uint8_t)idx);
    }

    refresh_pwm_timer_state();
}

void EncoderMotor_SetSpeedById(uint8_t motor_id, int16_t speed)
{
    int8_t idx;

    // BTS7960 route is single-motor debug path; map ALL to motor2.
    if (motor_id == ENCODER_MOTOR_ID_ALL) {
        motor_id = ENCODER_MOTOR_LEGACY_ID;
    }

    // Ignore other IDs on BTS7960 route to avoid pin conflicts.
    if (motor_id != ENCODER_MOTOR_LEGACY_ID) {
        return;
    }

    idx = id_to_index(motor_id);
    if (idx < 0) {
        return;
    }

    if (!g_state[(uint8_t)idx].enabled) {
        g_state[(uint8_t)idx].output_speed = 0;
        Set_Motor_Speed(0);
        return;
    }

    if (speed > 1000) speed = 1000;
    if (speed < -1000) speed = -1000;
    g_state[(uint8_t)idx].output_speed = speed;

    Set_Motor_Speed(speed);
    g_state[(uint8_t)idx].dir_forward = (speed >= 0) ? 1U : 0U;
}

void EncoderMotor_SetOutputPercentById(uint8_t motor_id, int16_t percent)
{
    if (percent > 100) percent = 100;
    if (percent < -100) percent = -100;
    EncoderMotor_SetSpeedById(motor_id, (int16_t)(percent * 10));
}

int16_t EncoderMotor_GetOutputPercentById(uint8_t motor_id)
{
    int8_t idx = id_to_index(motor_id);
    if (idx < 0) {
        return 0;
    }
    return (int16_t)(g_state[(uint8_t)idx].output_speed / 10);
}

uint8_t EncoderMotor_IsEnabledById(uint8_t motor_id)
{
    int8_t idx = id_to_index(motor_id);
    if (idx < 0) {
        return 0U;
    }
    return g_state[(uint8_t)idx].enabled;
}

uint8_t EncoderMotor_GetDirectionForwardById(uint8_t motor_id)
{
    int8_t idx = id_to_index(motor_id);
    if (idx < 0) {
        return 1U;
    }
    return g_state[(uint8_t)idx].dir_forward;
}

uint16_t EncoderMotor_GetPulseHzById(uint8_t motor_id)
{
    (void)motor_id;
    return PWM_FIXED_HZ;
}

int32_t EncoderMotor_GetEncoderCountById(uint8_t motor_id)
{
    int8_t idx = id_to_index(motor_id);
    if (idx < 0) {
        return 0;
    }
    return (int16_t)TIM_GetCounter(g_hw[(uint8_t)idx].encoder_timer);
}

void EncoderMotor_ResetEncoderById(uint8_t motor_id)
{
    int8_t idx;

    if (motor_id == ENCODER_MOTOR_ID_ALL) {
        EncoderMotor_ResetEncoderById(ENCODER_MOTOR_ID_1);
        EncoderMotor_ResetEncoderById(ENCODER_MOTOR_ID_2);
        EncoderMotor_ResetEncoderById(ENCODER_MOTOR_ID_3);
        return;
    }

    idx = id_to_index(motor_id);
    if (idx < 0) {
        return;
    }

    TIM_SetCounter(g_hw[(uint8_t)idx].encoder_timer, 0U);
}

void EncoderMotor_EnableSync12(uint8_t en)
{
    EncoderMotor_EnableById(ENCODER_MOTOR_ID_1, en);
    EncoderMotor_EnableById(ENCODER_MOTOR_ID_2, en);
}

void EncoderMotor_SetSync12OutputPercent(int16_t percent)
{
    EncoderMotor_SetOutputPercentById(ENCODER_MOTOR_ID_1, percent);
    EncoderMotor_SetOutputPercentById(ENCODER_MOTOR_ID_2, percent);
}

void EncoderMotor_Init(void)
{
    EncoderMotor_InitAll();
}

void EncoderMotor_Enable(uint8_t en)
{
    EncoderMotor_EnableById(ENCODER_MOTOR_LEGACY_ID, en);
}

void EncoderMotor_SetOutputPercent(int16_t percent)
{
    EncoderMotor_SetOutputPercentById(ENCODER_MOTOR_LEGACY_ID, percent);
}

int16_t EncoderMotor_GetOutputPercent(void)
{
    return EncoderMotor_GetOutputPercentById(ENCODER_MOTOR_LEGACY_ID);
}

uint8_t EncoderMotor_IsEnabled(void)
{
    return EncoderMotor_IsEnabledById(ENCODER_MOTOR_LEGACY_ID);
}

uint8_t EncoderMotor_GetDirectionForward(void)
{
    return EncoderMotor_GetDirectionForwardById(ENCODER_MOTOR_LEGACY_ID);
}

uint16_t EncoderMotor_GetPulseHz(void)
{
    return EncoderMotor_GetPulseHzById(ENCODER_MOTOR_LEGACY_ID);
}

int32_t EncoderMotor_GetEncoderCount(void)
{
    return EncoderMotor_GetEncoderCountById(ENCODER_MOTOR_LEGACY_ID);
}

void EncoderMotor_ResetEncoder(void)
{
    EncoderMotor_ResetEncoderById(ENCODER_MOTOR_LEGACY_ID);
}
