#include "dm4310_can.h"

#include "delay.h"

#define DM4310_CAN CAN1

#define DM4310_P_MIN (-12.5f)
#define DM4310_P_MAX (12.5f)
#define DM4310_V_MIN (-30.0f)
#define DM4310_V_MAX (30.0f)
#define DM4310_T_MIN (-10.0f)
#define DM4310_T_MAX (10.0f)
#define DM4310_KP_MIN (0.0f)
#define DM4310_KP_MAX (500.0f)
#define DM4310_KD_MIN (0.0f)
#define DM4310_KD_MAX (5.0f)

#define DM4310_CMD_ENABLE_LAST 0xFC
#define DM4310_CMD_DISABLE_LAST 0xFD
#define DM4310_CMD_SAVE_ZERO_LAST 0xFE
#define DM4310_CMD_CLEAR_ERR_LAST 0xFB

static uint16_t g_motor_id = 1;
static uint8_t g_can_ready = 0;

static float g_p_des = 0.0f;
static float g_v_des = 0.0f;
static float g_kp = 0.0f;
static float g_kd = 1.0f;
static float g_t_ff = 0.0f;

static volatile DM4310_Feedback_t g_fb;

static float dm_clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static uint16_t dm_float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span;
    float max_int;

    x = dm_clampf(x, x_min, x_max);
    span = x_max - x_min;
    max_int = (float)((1UL << bits) - 1UL);
    return (uint16_t)(((x - x_min) * max_int) / span);
}

static float dm_uint_to_float(uint16_t x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float max_int = (float)((1UL << bits) - 1UL);
    return ((float)x) * span / max_int + x_min;
}

static void dm_pack_mit(uint8_t out[8], float p_des, float v_des, float kp, float kd, float t_ff)
{
    uint16_t p_uint = dm_float_to_uint(p_des, DM4310_P_MIN, DM4310_P_MAX, 16);
    uint16_t v_uint = dm_float_to_uint(v_des, DM4310_V_MIN, DM4310_V_MAX, 12);
    uint16_t kp_uint = dm_float_to_uint(kp, DM4310_KP_MIN, DM4310_KP_MAX, 12);
    uint16_t kd_uint = dm_float_to_uint(kd, DM4310_KD_MIN, DM4310_KD_MAX, 12);
    uint16_t t_uint = dm_float_to_uint(t_ff, DM4310_T_MIN, DM4310_T_MAX, 12);

    out[0] = (uint8_t)((p_uint >> 8) & 0xFF);
    out[1] = (uint8_t)(p_uint & 0xFF);
    out[2] = (uint8_t)((v_uint >> 4) & 0xFF);
    out[3] = (uint8_t)(((v_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F));
    out[4] = (uint8_t)(kp_uint & 0xFF);
    out[5] = (uint8_t)((kd_uint >> 4) & 0xFF);
    out[6] = (uint8_t)(((kd_uint & 0x0F) << 4) | ((t_uint >> 8) & 0x0F));
    out[7] = (uint8_t)(t_uint & 0xFF);
}

static void dm_make_special_cmd(uint8_t out[8], uint8_t last_byte)
{
    out[0] = 0xFF;
    out[1] = 0xFF;
    out[2] = 0xFF;
    out[3] = 0xFF;
    out[4] = 0xFF;
    out[5] = 0xFF;
    out[6] = 0xFF;
    out[7] = last_byte;
}

static void dm_make_write_u32(uint8_t out[8], uint16_t can_id, uint8_t rid, uint32_t value)
{
    out[0] = (uint8_t)(can_id & 0xFF);
    out[1] = (uint8_t)((can_id >> 8) & 0xFF);
    out[2] = 0x55;
    out[3] = rid;
    out[4] = (uint8_t)(value & 0xFF);
    out[5] = (uint8_t)((value >> 8) & 0xFF);
    out[6] = (uint8_t)((value >> 16) & 0xFF);
    out[7] = (uint8_t)((value >> 24) & 0xFF);
}

static uint8_t dm_can_send_std(uint16_t std_id, const uint8_t data[8])
{
    CanTxMsg tx;
    uint8_t mailbox;
    uint32_t timeout;
    uint8_t status;
    uint8_t i;

    if (!g_can_ready) {
        return 0;
    }

    tx.StdId = std_id;
    tx.ExtId = 0;
    tx.IDE = CAN_Id_Standard;
    tx.RTR = CAN_RTR_Data;
    tx.DLC = 8;

    for (i = 0; i < 8; i++) {
        tx.Data[i] = data[i];
    }

    mailbox = CAN_Transmit(DM4310_CAN, &tx);
    if (mailbox > 2U) {
        return 0;
    }

    timeout = 0;
    while (timeout < 200000UL) {
        status = CAN_TransmitStatus(DM4310_CAN, mailbox);
        if (status == CAN_TxStatus_Ok) {
            return 1;
        }
        if (status == CAN_TxStatus_Failed) {
            return 0;
        }
        timeout++;
    }

    return 0;
}

static void dm_parse_feedback(const uint8_t d[8])
{
    uint16_t p_uint;
    uint16_t v_uint;
    uint16_t t_uint;

    p_uint = (uint16_t)(((uint16_t)d[1] << 8) | d[2]);
    v_uint = (uint16_t)(((uint16_t)d[3] << 4) | ((uint16_t)d[4] >> 4));
    t_uint = (uint16_t)((((uint16_t)d[4] & 0x0F) << 8) | d[5]);

    g_fb.valid = 1;
    g_fb.err = (uint8_t)((d[0] >> 4) & 0x0F);
    g_fb.motor_id = (uint8_t)(d[0] & 0x0F);
    g_fb.pos_rad = dm_uint_to_float(p_uint, DM4310_P_MIN, DM4310_P_MAX, 16);
    g_fb.vel_rad_s = dm_uint_to_float(v_uint, DM4310_V_MIN, DM4310_V_MAX, 12);
    g_fb.tau_nm = dm_uint_to_float(t_uint, DM4310_T_MIN, DM4310_T_MAX, 12);
    g_fb.mos_temp = d[6];
    g_fb.rotor_temp = d[7];
    g_fb.rx_count++;
}

static void dm_add_unique_id(uint16_t *ids, uint8_t *count, uint16_t value)
{
    uint8_t i;
    for (i = 0; i < *count; i++) {
        if (ids[i] == value) {
            return;
        }
    }
    if (*count < 8U) {
        ids[*count] = value;
        (*count)++;
    }
}

static uint8_t dm_build_candidate_ids(uint16_t base_id, uint16_t *ids)
{
    uint16_t mode;
    uint8_t count = 0;

    dm_add_unique_id(ids, &count, base_id);
    dm_add_unique_id(ids, &count, (uint16_t)(0x100U + base_id));
    dm_add_unique_id(ids, &count, (uint16_t)(0x200U + base_id));
    dm_add_unique_id(ids, &count, (uint16_t)(0x300U + base_id));

    for (mode = 1; mode <= 4; mode++) {
        dm_add_unique_id(ids, &count, (uint16_t)(((mode - 1U) << 2U) + base_id));
    }

    return count;
}

static uint8_t dm_can_hw_init(void)
{
    GPIO_InitTypeDef gpio;
    CAN_InitTypeDef can_init;
    CAN_FilterInitTypeDef filter;
    NVIC_InitTypeDef nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_PinRemapConfig(GPIO_Remap1_CAN1, DISABLE);
    GPIO_PinRemapConfig(GPIO_Remap2_CAN1, DISABLE);

    gpio.GPIO_Pin = GPIO_Pin_12;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);

    CAN_DeInit(DM4310_CAN);
    CAN_StructInit(&can_init);

    can_init.CAN_TTCM = DISABLE;
    can_init.CAN_ABOM = ENABLE;
    can_init.CAN_AWUM = DISABLE;
    can_init.CAN_NART = DISABLE;
    can_init.CAN_RFLM = DISABLE;
    can_init.CAN_TXFP = DISABLE;
    can_init.CAN_Mode = CAN_Mode_Normal;

    // APB1=36MHz -> 36/(4*(1+6+2)) = 1MHz
    can_init.CAN_SJW = CAN_SJW_1tq;
    can_init.CAN_BS1 = CAN_BS1_6tq;
    can_init.CAN_BS2 = CAN_BS2_2tq;
    can_init.CAN_Prescaler = 4;

    if (CAN_Init(DM4310_CAN, &can_init) != CAN_InitStatus_Success) {
        return 0;
    }

    filter.CAN_FilterNumber = 0;
    filter.CAN_FilterMode = CAN_FilterMode_IdMask;
    filter.CAN_FilterScale = CAN_FilterScale_32bit;
    filter.CAN_FilterIdHigh = 0;
    filter.CAN_FilterIdLow = 0;
    filter.CAN_FilterMaskIdHigh = 0;
    filter.CAN_FilterMaskIdLow = 0;
    filter.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    filter.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&filter);

    nvic.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    CAN_ITConfig(DM4310_CAN, CAN_IT_FMP0, ENABLE);

    return 1;
}

uint8_t DM4310_Init(uint16_t motor_id)
{
    g_motor_id = motor_id;
    g_p_des = 0.0f;
    g_v_des = 0.0f;
    g_kp = 0.0f;
    g_kd = 1.0f;
    g_t_ff = 0.0f;

    g_fb.valid = 0;
    g_fb.motor_id = 0;
    g_fb.err = 0;
    g_fb.pos_rad = 0.0f;
    g_fb.vel_rad_s = 0.0f;
    g_fb.tau_nm = 0.0f;
    g_fb.mos_temp = 0;
    g_fb.rotor_temp = 0;
    g_fb.rx_count = 0;

    g_can_ready = dm_can_hw_init();
    return g_can_ready;
}

void DM4310_SetMotorId(uint16_t motor_id)
{
    g_motor_id = motor_id;
}

uint16_t DM4310_GetMotorId(void)
{
    return g_motor_id;
}

void DM4310_SetMitCommand(float p_des, float v_des, float kp, float kd, float t_ff)
{
    g_p_des = dm_clampf(p_des, DM4310_P_MIN, DM4310_P_MAX);
    g_v_des = dm_clampf(v_des, DM4310_V_MIN, DM4310_V_MAX);
    g_kp = dm_clampf(kp, DM4310_KP_MIN, DM4310_KP_MAX);
    g_kd = dm_clampf(kd, DM4310_KD_MIN, DM4310_KD_MAX);
    g_t_ff = dm_clampf(t_ff, DM4310_T_MIN, DM4310_T_MAX);
}

uint8_t DM4310_SendMitNow(void)
{
    uint8_t mit[8];
    dm_pack_mit(mit, g_p_des, g_v_des, g_kp, g_kd, g_t_ff);
    return dm_can_send_std(g_motor_id, mit);
}

void DM4310_EnterMitMode(void)
{
    uint8_t cmd[8];
    dm_make_write_u32(cmd, g_motor_id, 0x0A, 1UL);
    dm_can_send_std(0x7FF, cmd);
}

void DM4310_EnableCompat(void)
{
    uint16_t ids[8];
    uint8_t count;
    uint8_t i;
    uint8_t k;
    uint8_t cmd_en[8];
    uint8_t cmd_dis[8];
    uint8_t cmd_clr[8];

    count = dm_build_candidate_ids(g_motor_id, ids);

    dm_make_special_cmd(cmd_en, DM4310_CMD_ENABLE_LAST);
    dm_make_special_cmd(cmd_dis, DM4310_CMD_DISABLE_LAST);
    dm_make_special_cmd(cmd_clr, DM4310_CMD_CLEAR_ERR_LAST);

    for (i = 0; i < count; i++) {
        dm_can_send_std(ids[i], cmd_clr);
        dm_can_send_std(ids[i], cmd_dis);
    }

    delay_ms(20);

    for (k = 0; k < 3U; k++) {
        for (i = 0; i < count; i++) {
            dm_can_send_std(ids[i], cmd_en);
        }
        delay_ms(5);
    }
}

void DM4310_Disable(void)
{
    uint8_t cmd[8];
    dm_make_special_cmd(cmd, DM4310_CMD_DISABLE_LAST);
    dm_can_send_std(g_motor_id, cmd);
}

void DM4310_ClearError(void)
{
    uint8_t cmd[8];
    dm_make_special_cmd(cmd, DM4310_CMD_CLEAR_ERR_LAST);
    dm_can_send_std(g_motor_id, cmd);
}

void DM4310_SaveZero(void)
{
    uint8_t cmd[8];
    dm_make_special_cmd(cmd, DM4310_CMD_SAVE_ZERO_LAST);
    dm_can_send_std(g_motor_id, cmd);
}

uint8_t DM4310_HasFeedback(void)
{
    return g_fb.valid;
}

uint8_t DM4310_GetFeedback(DM4310_Feedback_t *out)
{
    if (out == 0) {
        return 0;
    }
    if (!g_fb.valid) {
        return 0;
    }
    *out = g_fb;
    return 1;
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
    CanRxMsg rx;

    while (CAN_MessagePending(DM4310_CAN, CAN_FIFO0) > 0U) {
        CAN_Receive(DM4310_CAN, CAN_FIFO0, &rx);
        if ((rx.IDE == CAN_Id_Standard) && (rx.DLC >= 8U)) {
            dm_parse_feedback(rx.Data);
        }
    }
}
