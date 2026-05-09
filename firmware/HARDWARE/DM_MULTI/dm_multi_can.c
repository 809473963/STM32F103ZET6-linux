#include "dm_multi_can.h"

#include <string.h>

#include "delay.h"

#define DM_MULTI_CAN CAN1

#define DM_P_MIN (-12.5f)
#define DM_P_MAX (12.5f)
#define DM_KP_MIN (0.0f)
#define DM_KP_MAX (500.0f)
#define DM_KD_MIN (0.0f)
#define DM_KD_MAX (5.0f)

#define DM_CMD_ENABLE_LAST 0xFCU
#define DM_CMD_DISABLE_LAST 0xFDU
#define DM_CMD_SAVE_ZERO_LAST 0xFEU
#define DM_CMD_CLEAR_ERR_LAST 0xFBU

static DM_MultiMotorConfig_t g_cfg[DM_MULTI_MAX_MOTORS];
static volatile DM_MultiFeedback_t g_fb[DM_MULTI_MAX_MOTORS];
static volatile DM_MultiRegAck_t g_last_ack;

static uint8_t g_cfg_count = 0U;
static uint8_t g_can_ready = 0U;
static volatile uint32_t g_seen_id_mask = 0U;

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

static void dm_pack_mit(uint8_t out[8], float q_max, float dq_max, float tau_max,
                        float p_des, float v_des, float kp, float kd, float t_ff)
{
    uint16_t p_uint;
    uint16_t v_uint;
    uint16_t kp_uint;
    uint16_t kd_uint;
    uint16_t t_uint;

    if (q_max <= 0.1f) q_max = 12.5f;
    if (dq_max <= 0.1f) dq_max = 30.0f;
    if (tau_max <= 0.1f) tau_max = 10.0f;

    p_uint = dm_float_to_uint(p_des, -q_max, q_max, 16);
    v_uint = dm_float_to_uint(v_des, -dq_max, dq_max, 12);
    kp_uint = dm_float_to_uint(kp, DM_KP_MIN, DM_KP_MAX, 12);
    kd_uint = dm_float_to_uint(kd, DM_KD_MIN, DM_KD_MAX, 12);
    t_uint = dm_float_to_uint(t_ff, -tau_max, tau_max, 12);

    out[0] = (uint8_t)((p_uint >> 8) & 0xFFU);
    out[1] = (uint8_t)(p_uint & 0xFFU);
    out[2] = (uint8_t)((v_uint >> 4) & 0xFFU);
    out[3] = (uint8_t)(((v_uint & 0x0FU) << 4) | ((kp_uint >> 8) & 0x0FU));
    out[4] = (uint8_t)(kp_uint & 0xFFU);
    out[5] = (uint8_t)((kd_uint >> 4) & 0xFFU);
    out[6] = (uint8_t)(((kd_uint & 0x0FU) << 4) | ((t_uint >> 8) & 0x0FU));
    out[7] = (uint8_t)(t_uint & 0xFFU);
}

static void dm_make_special_cmd(uint8_t out[8], uint8_t last_byte)
{
    out[0] = 0xFFU;
    out[1] = 0xFFU;
    out[2] = 0xFFU;
    out[3] = 0xFFU;
    out[4] = 0xFFU;
    out[5] = 0xFFU;
    out[6] = 0xFFU;
    out[7] = last_byte;
}

static void dm_make_reg_cmd(uint8_t out[8], uint16_t can_id, uint8_t op, uint8_t rid, uint32_t value)
{
    out[0] = (uint8_t)(can_id & 0xFFU);
    out[1] = (uint8_t)((can_id >> 8) & 0xFFU);
    out[2] = op;
    out[3] = rid;
    out[4] = (uint8_t)(value & 0xFFU);
    out[5] = (uint8_t)((value >> 8) & 0xFFU);
    out[6] = (uint8_t)((value >> 16) & 0xFFU);
    out[7] = (uint8_t)((value >> 24) & 0xFFU);
}

static void dm_make_status_req(uint8_t out[8], uint16_t can_id)
{
    out[0] = (uint8_t)(can_id & 0xFFU);
    out[1] = (uint8_t)((can_id >> 8) & 0xFFU);
    out[2] = 0xCCU;
    out[3] = 0x00U;
    out[4] = 0x00U;
    out[5] = 0x00U;
    out[6] = 0x00U;
    out[7] = 0x00U;
}

static void dm_default_limits(DM_MultiMotorType_t type, float *q_max, float *dq_max, float *tau_max)
{
    if (q_max == 0 || dq_max == 0 || tau_max == 0) {
        return;
    }

    switch (type) {
        case DM_MULTI_TYPE_DM3510:
            *q_max = 12.5f;
            *dq_max = 280.0f;
            *tau_max = 1.0f;
            break;
        case DM_MULTI_TYPE_M24:
            *q_max = 12.5f;
            *dq_max = 80.0f;
            *tau_max = 8.0f;
            break;
        case DM_MULTI_TYPE_DM4310:
        default:
            *q_max = 12.5f;
            *dq_max = 30.0f;
            *tau_max = 10.0f;
            break;
    }
}

static int8_t dm_find_index_by_can_id(uint16_t can_id)
{
    uint8_t i;
    for (i = 0U; i < g_cfg_count; i++) {
        if (g_cfg[i].can_id == can_id) {
            return (int8_t)i;
        }
        if ((uint16_t)(g_cfg[i].can_id + 0x10U) == can_id) {
            return (int8_t)i;
        }
    }
    return -1;
}

static int8_t dm_find_index_by_low4(uint8_t low4_id)
{
    uint8_t i;
    for (i = 0U; i < g_cfg_count; i++) {
        if ((g_cfg[i].can_id & 0x0FU) == low4_id) {
            return (int8_t)i;
        }
    }
    return -1;
}

static uint8_t dm_can_send_std(uint16_t std_id, const uint8_t data[8])
{
    CanTxMsg tx;
    uint8_t mailbox;
    uint32_t timeout;
    uint8_t status;
    uint8_t i;

    if (!g_can_ready) {
        return 0U;
    }

    tx.StdId = std_id;
    tx.ExtId = 0U;
    tx.IDE = CAN_Id_Standard;
    tx.RTR = CAN_RTR_Data;
    tx.DLC = 8U;

    for (i = 0U; i < 8U; i++) {
        tx.Data[i] = data[i];
    }

    mailbox = CAN_Transmit(DM_MULTI_CAN, &tx);
    if (mailbox > 2U) {
        return 0U;
    }

    // Fast-fail: 2000 iterations (~100us at 72MHz) is enough for ACK on healthy bus.
    // Without a termination resistor or motor, CAN_TxStatus_Failed arrives in 1-5 iterations.
    timeout = 0U;
    while (timeout < 2000UL) {
        status = CAN_TransmitStatus(DM_MULTI_CAN, mailbox);
        if (status == CAN_TxStatus_Ok) {
            return 1U;
        }
        if (status == CAN_TxStatus_Failed) {
            return 0U;
        }
        timeout++;
    }

    CAN_CancelTransmit(DM_MULTI_CAN, mailbox);
    return 0U;
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

    CAN_DeInit(DM_MULTI_CAN);
    CAN_StructInit(&can_init);

    can_init.CAN_TTCM = DISABLE;
    can_init.CAN_ABOM = ENABLE;
    can_init.CAN_AWUM = DISABLE;
    can_init.CAN_NART = DISABLE;
    can_init.CAN_RFLM = DISABLE;
    can_init.CAN_TXFP = DISABLE;
    can_init.CAN_Mode = CAN_Mode_Normal;

    can_init.CAN_SJW = CAN_SJW_1tq;
    can_init.CAN_BS1 = CAN_BS1_6tq;
    can_init.CAN_BS2 = CAN_BS2_2tq;
    can_init.CAN_Prescaler = 4;

    if (CAN_Init(DM_MULTI_CAN, &can_init) != CAN_InitStatus_Success) {
        return 0U;
    }

    filter.CAN_FilterNumber = 0;
    filter.CAN_FilterMode = CAN_FilterMode_IdMask;
    filter.CAN_FilterScale = CAN_FilterScale_32bit;
    filter.CAN_FilterIdHigh = 0U;
    filter.CAN_FilterIdLow = 0U;
    filter.CAN_FilterMaskIdHigh = 0U;
    filter.CAN_FilterMaskIdLow = 0U;
    filter.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    filter.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&filter);

    nvic.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    CAN_ITConfig(DM_MULTI_CAN, CAN_IT_FMP0, ENABLE);

    return 1U;
}

uint8_t DM_Multi_Init(const DM_MultiMotorConfig_t *cfg, uint8_t cfg_count)
{
    uint8_t i;

    if (cfg == 0 || cfg_count == 0U || cfg_count > DM_MULTI_MAX_MOTORS) {
        return 0U;
    }

    g_cfg_count = cfg_count;
    for (i = 0U; i < g_cfg_count; i++) {
        g_cfg[i] = cfg[i];

        if (g_cfg[i].q_max <= 0.1f || g_cfg[i].dq_max <= 0.1f || g_cfg[i].tau_max <= 0.1f) {
            dm_default_limits(g_cfg[i].type, &g_cfg[i].q_max, &g_cfg[i].dq_max, &g_cfg[i].tau_max);
        }

        g_fb[i].valid = 0U;
        g_fb[i].online = 0U;
        g_fb[i].can_id = g_cfg[i].can_id;
        g_fb[i].motor_id = 0U;
        g_fb[i].err = 0U;
        g_fb[i].pos_rad = 0.0f;
        g_fb[i].vel_rad_s = 0.0f;
        g_fb[i].tau_nm = 0.0f;
        g_fb[i].mos_temp = 0U;
        g_fb[i].rotor_temp = 0U;
        g_fb[i].rx_count = 0U;
    }

    g_last_ack.valid = 0U;
    g_last_ack.op = 0U;
    g_last_ack.can_id = 0U;
    g_last_ack.rid = 0U;
    g_last_ack.value_u32 = 0U;
    g_seen_id_mask = 0U;

    g_can_ready = dm_can_hw_init();
    return g_can_ready;
}

uint8_t DM_Multi_IsReady(void)
{
    return g_can_ready;
}

uint8_t DM_Multi_Enable(uint16_t can_id)
{
    uint8_t cmd[8];
    dm_make_special_cmd(cmd, DM_CMD_ENABLE_LAST);
    return dm_can_send_std(can_id, cmd);
}

uint8_t DM_Multi_Disable(uint16_t can_id)
{
    uint8_t cmd[8];
    dm_make_special_cmd(cmd, DM_CMD_DISABLE_LAST);
    return dm_can_send_std(can_id, cmd);
}

uint8_t DM_Multi_ClearError(uint16_t can_id)
{
    uint8_t cmd[8];
    dm_make_special_cmd(cmd, DM_CMD_CLEAR_ERR_LAST);
    return dm_can_send_std(can_id, cmd);
}

uint8_t DM_Multi_SaveZero(uint16_t can_id)
{
    uint8_t cmd[8];
    dm_make_special_cmd(cmd, DM_CMD_SAVE_ZERO_LAST);
    return dm_can_send_std(can_id, cmd);
}

uint8_t DM_Multi_EnableCompat(uint16_t can_id)
{
    uint16_t ids[8];
    uint8_t count = 0U;
    uint8_t i;
    uint8_t k;
    uint8_t ok = 0U;

    uint8_t cmd_en[8];
    uint8_t cmd_dis[8];
    uint8_t cmd_clr[8];

    ids[count++] = can_id;
    ids[count++] = (uint16_t)(0x100U + can_id);
    ids[count++] = (uint16_t)(0x200U + can_id);
    ids[count++] = (uint16_t)(0x300U + can_id);
    ids[count++] = (uint16_t)((0U << 2U) + can_id);
    ids[count++] = (uint16_t)((1U << 2U) + can_id);
    ids[count++] = (uint16_t)((2U << 2U) + can_id);
    ids[count++] = (uint16_t)((3U << 2U) + can_id);

    dm_make_special_cmd(cmd_en, DM_CMD_ENABLE_LAST);
    dm_make_special_cmd(cmd_dis, DM_CMD_DISABLE_LAST);
    dm_make_special_cmd(cmd_clr, DM_CMD_CLEAR_ERR_LAST);

    for (i = 0U; i < count; i++) {
        dm_can_send_std(ids[i], cmd_clr);
        dm_can_send_std(ids[i], cmd_dis);
    }

    delay_ms(20);

    for (k = 0U; k < 3U; k++) {
        for (i = 0U; i < count; i++) {
            ok = (uint8_t)(dm_can_send_std(ids[i], cmd_en) || ok);
        }
        delay_ms(5);
    }

    return ok;
}

uint8_t DM_Multi_SetControlModeMit(uint16_t can_id)
{
    return DM_Multi_WriteRegisterU32(can_id, DM_MULTI_RID_CTRL_MODE, 1UL);
}

uint8_t DM_Multi_SetControlModeVel(uint16_t can_id)
{
    return DM_Multi_WriteRegisterU32(can_id, DM_MULTI_RID_CTRL_MODE, DM_CTRL_MODE_VEL);
}

uint8_t DM_Multi_SetVel(uint16_t can_id, float vel)
{
    uint8_t data[8] = {0};
    memcpy(&data[0], &vel, sizeof(float));
    return dm_can_send_std((uint16_t)(can_id + DM_SPEED_MODE_ID_OFFSET), data);
}

uint8_t DM_Multi_SetMit(uint16_t can_id, float p_des, float v_des, float kp, float kd, float t_ff)
{
    int8_t idx;
    uint8_t mit[8];

    idx = dm_find_index_by_can_id(can_id);
    if (idx < 0) {
        idx = dm_find_index_by_low4((uint8_t)(can_id & 0x0FU));
        if (idx < 0) {
            return 0U;
        }
    }

    dm_pack_mit(
        mit,
        g_cfg[idx].q_max,
        g_cfg[idx].dq_max,
        g_cfg[idx].tau_max,
        p_des,
        v_des,
        kp,
        kd,
        t_ff
    );

    return dm_can_send_std(can_id, mit);
}

uint8_t DM_Multi_RequestStatus(uint16_t can_id)
{
    uint8_t req[8];
    dm_make_status_req(req, can_id);
    return dm_can_send_std(0x7FFU, req);
}

uint8_t DM_Multi_ReadRegister(uint16_t can_id, uint8_t rid)
{
    uint8_t req[8];
    dm_make_reg_cmd(req, can_id, 0x33U, rid, 0UL);
    return dm_can_send_std(0x7FFU, req);
}

uint8_t DM_Multi_WriteRegisterU32(uint16_t can_id, uint8_t rid, uint32_t value)
{
    uint8_t req[8];
    dm_make_reg_cmd(req, can_id, 0x55U, rid, value);
    return dm_can_send_std(0x7FFU, req);
}

uint8_t DM_Multi_SendRawCAN(uint16_t std_id, const uint8_t data[8])
{
    return dm_can_send_std(std_id, data);
}

uint8_t DM_Multi_SetMotorId(uint16_t current_id, uint16_t new_id, uint8_t update_feedback_id)
{
    uint8_t ok;
    ok = DM_Multi_WriteRegisterU32(current_id, DM_MULTI_RID_RECV_ID, (uint32_t)new_id);
    if (update_feedback_id) {
        ok = (uint8_t)(DM_Multi_WriteRegisterU32(current_id, DM_MULTI_RID_FEEDBACK_ID,
                                                (uint32_t)(new_id + 0x10U)) && ok);
    }
    return ok;
}

uint8_t DM_Multi_GetFeedback(uint16_t can_id, DM_MultiFeedback_t *out)
{
    int8_t idx;

    if (out == 0) {
        return 0U;
    }

    idx = dm_find_index_by_can_id(can_id);
    if (idx < 0) {
        idx = dm_find_index_by_low4((uint8_t)(can_id & 0x0FU));
        if (idx < 0) {
            return 0U;
        }
    }

    if (!g_fb[idx].valid) {
        return 0U;
    }

    *out = (DM_MultiFeedback_t)g_fb[idx];
    return 1U;
}

uint8_t DM_Multi_GetFeedbackByIndex(uint8_t index, DM_MultiFeedback_t *out)
{
    if (out == 0 || index >= g_cfg_count) {
        return 0U;
    }
    if (!g_fb[index].valid) {
        return 0U;
    }
    *out = (DM_MultiFeedback_t)g_fb[index];
    return 1U;
}

void DM_Multi_ClearSeenIds(void)
{
    g_seen_id_mask = 0U;
}

uint8_t DM_Multi_GetSeenIds(uint8_t *out_ids, uint8_t max_count)
{
    uint8_t count = 0U;
    uint8_t i;
    uint32_t mask = g_seen_id_mask;

    if (out_ids == 0 || max_count == 0U) {
        return 0U;
    }

    for (i = 1U; i < 32U; i++) {
        if ((mask & (1UL << i)) != 0UL) {
            out_ids[count++] = i;
            if (count >= max_count) {
                break;
            }
        }
    }

    return count;
}

uint32_t DM_Multi_GetSeenIdMask(void)
{
    return g_seen_id_mask;
}

void DM_Multi_ClearLastRegAck(void)
{
    g_last_ack.valid = 0U;
    g_last_ack.op = 0U;
    g_last_ack.can_id = 0U;
    g_last_ack.rid = 0U;
    g_last_ack.value_u32 = 0U;
}

uint8_t DM_Multi_GetLastRegAck(DM_MultiRegAck_t *out)
{
    if (out == 0) {
        return 0U;
    }
    if (!g_last_ack.valid) {
        return 0U;
    }
    *out = (DM_MultiRegAck_t)g_last_ack;
    return 1U;
}

static void dm_parse_feedback(uint16_t std_id, const uint8_t d[8])
{
    int8_t idx;
    uint16_t p_uint;
    uint16_t v_uint;
    uint16_t t_uint;
    uint8_t low4;

    low4 = (uint8_t)(d[0] & 0x0FU);

    if (std_id < 32U) {
        g_seen_id_mask |= (1UL << std_id);
    }
    if (low4 < 32U) {
        g_seen_id_mask |= (1UL << low4);
    }

    idx = dm_find_index_by_can_id(std_id);
    if (idx < 0) {
        idx = dm_find_index_by_low4(low4);
    }
    if (idx < 0) {
        return;
    }

    p_uint = (uint16_t)(((uint16_t)d[1] << 8) | d[2]);
    v_uint = (uint16_t)(((uint16_t)d[3] << 4) | ((uint16_t)d[4] >> 4));
    t_uint = (uint16_t)((((uint16_t)d[4] & 0x0FU) << 8) | d[5]);

    g_fb[idx].valid = 1U;
    g_fb[idx].online = 1U;
    g_fb[idx].can_id = g_cfg[idx].can_id;
    g_fb[idx].motor_id = low4;
    g_fb[idx].err = (uint8_t)((d[0] >> 4) & 0x0FU);
    g_fb[idx].pos_rad = dm_uint_to_float(p_uint, -g_cfg[idx].q_max, g_cfg[idx].q_max, 16);
    g_fb[idx].vel_rad_s = dm_uint_to_float(v_uint, -g_cfg[idx].dq_max, g_cfg[idx].dq_max, 12);
    g_fb[idx].tau_nm = dm_uint_to_float(t_uint, -g_cfg[idx].tau_max, g_cfg[idx].tau_max, 12);
    g_fb[idx].mos_temp = d[6];
    g_fb[idx].rotor_temp = d[7];
    g_fb[idx].rx_count++;
}

static void dm_parse_reg_ack(const uint8_t d[8])
{
    uint16_t can_id = (uint16_t)((((uint16_t)d[1]) << 8) | d[0]);

    if (can_id < 32U) {
        g_seen_id_mask |= (1UL << can_id);
    }

    g_last_ack.valid = 1U;
    g_last_ack.op = d[2];
    g_last_ack.can_id = can_id;
    g_last_ack.rid = d[3];
    g_last_ack.value_u32 = ((uint32_t)d[4]) |
                           (((uint32_t)d[5]) << 8) |
                           (((uint32_t)d[6]) << 16) |
                           (((uint32_t)d[7]) << 24);
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
    CanRxMsg rx;

    while (CAN_MessagePending(DM_MULTI_CAN, CAN_FIFO0) > 0U) {
        CAN_Receive(DM_MULTI_CAN, CAN_FIFO0, &rx);

        if (rx.IDE != CAN_Id_Standard || rx.DLC < 8U) {
            continue;
        }

        /* Register ack: motor may reply on 0x7FF or on its master_id.
         * Check d[2] for op code 0x33(read) / 0x55(write). */
        if (rx.Data[2] == 0x33U || rx.Data[2] == 0x55U) {
            /* Extra validation: d[0:2] = can_id(LE), which should be a
             * small number (1-16 typical).  Feedback d[0] upper nibble
             * is error flags, d[1] is position MSB – unlikely both to
             * look like a valid reg ack when g_last_ack was just cleared. */
            uint16_t ack_id = (uint16_t)(((uint16_t)rx.Data[1] << 8) | rx.Data[0]);
            if (ack_id > 0U && ack_id <= 32U && !g_last_ack.valid) {
                dm_parse_reg_ack(rx.Data);
                continue;
            }
        }

        if (rx.StdId == 0x7FFU) {
            continue;  /* non-reg 0x7FF frame, ignore */
        }

        dm_parse_feedback(rx.StdId, rx.Data);
    }
}
