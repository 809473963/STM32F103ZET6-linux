#include "protocol.h"

#include "delay.h"
#include "dm_multi_can.h"
#include <math.h>
#include <string.h>

#include "led.h"
#include "usart.h"

static ProtocolState_t rx_state = STATE_WAIT_HEADER;
static ProtocolFrame_t current_frame;
static uint8_t rx_data_index = 0;
static uint8_t calc_checksum = 0;

#ifndef DIR_STOP
#define DIR_STOP 2
#endif

#define ARM_GATEWAY_FRAME_LEN 39U
#define ARM_STATUS_PAYLOAD_LEN 17U

#define ARM_GROUP_A 1U
#define ARM_MODE_MIT 1U

#define ARM_MOTOR_COUNT 2U
#define ARM_SERVICE_PERIOD_MS 10U
#define ARM_STATUS_POLL_PERIOD_MS 50U
#define ARM_COMM_WATCHDOG_MS 500U   /* auto-stop if no valid frame in this period */

typedef struct {
    uint8_t enabled;
    uint8_t control_mode;
    uint8_t motor_group;
    float p_des;
    float v_des;
    float kp;
    float kd;
    float t_ff;
    float target_position;
    float target_velocity;
} ArmMotorCmd_t;

typedef struct {
    uint8_t enabled;
    uint8_t zero_initialized;
    float zero_pos_rad;      // "flat arm" startup reference
    float kg;                // gravity gain
    float bias_rad;          // phase/bias
    float viscous;           // velocity damping feedforward
    float coulomb;           // static friction feedforward
} ArmGravityComp_t;

static const DM_MultiMotorConfig_t g_arm_motor_cfg[ARM_MOTOR_COUNT] = {
    // ID1(DM4310) -> arm base (MIT mode)
    // ID2(DMH3510) -> arm joint1 (VEL mode)
    {1U, DM_MULTI_TYPE_DM4310, 12.5f, 30.0f, 10.0f},
    {2U, DM_MULTI_TYPE_DM3510, 12.5f, 280.0f, 1.0f}
};

static const float g_arm_full_scale_vel[ARM_MOTOR_COUNT] = {6.0f, 12.0f};

static ArmMotorCmd_t g_arm_cmd[ARM_MOTOR_COUNT];
static uint8_t g_arm_applied_enable[ARM_MOTOR_COUNT];
static ArmGravityComp_t g_arm_grav[ARM_MOTOR_COUNT];

static uint8_t g_gateway_buf[ARM_GATEWAY_FRAME_LEN];
static uint8_t g_gateway_index = 0U;

static uint16_t g_arm_service_elapsed_ms = 0U;
static uint16_t g_arm_status_elapsed_ms = 0U;
static uint16_t g_arm_beacon_elapsed_ms = 0U;
static uint16_t g_arm_comm_watchdog_ms = 0U;
static uint8_t  g_arm_watchdog_tripped = 0U;
static uint8_t g_status_focus_index = 0U;

static void arm_send_frame(uint8_t cmd, const uint8_t *payload, uint8_t length);

static float arm_absf(float x) { return (x >= 0.0f) ? x : -x; }
static float arm_signf(float x) { return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f); }

static float arm_gravity_ff(uint8_t index, const DM_MultiFeedback_t *fb)
{
    ArmGravityComp_t *gc;
    float theta_local;
    float ff;

    if (index >= ARM_MOTOR_COUNT || fb == 0) {
        return 0.0f;
    }

    gc = &g_arm_grav[index];
    if (!gc->enabled) {
        return 0.0f;
    }

    if (!gc->zero_initialized) {
        /* zero_pos_rad has not been calibrated yet (user has not pressed C).
         * Return 0 so the motor is held by KP/KD alone until the user
         * explicitly presses C while the arm is at horizontal. */
        return 0.0f;
    }

    theta_local = fb->pos_rad - gc->zero_pos_rad;
    /* Gravity model: T_ff = KG * cos(θ_local + bias).
     * zero_pos is set to the horizontal position (C key in host workflow) so cos(0) = 1
     * gives maximum compensation there, falling to 0 at vertical.
     * bias trims residual zero-offset; no need to set bias = π/2. */
    ff = gc->kg * cosf(theta_local + gc->bias_rad);
    ff += gc->viscous * fb->vel_rad_s;
    if (arm_absf(fb->vel_rad_s) < 0.02f) {
        ff += gc->coulomb * arm_signf(g_arm_cmd[index].v_des);
    } else {
        ff += gc->coulomb * arm_signf(fb->vel_rad_s);
    }
    return ff;
}

static int8_t arm_index_from_motor_id(uint8_t motor_id)
{
    if (motor_id >= 1U && motor_id <= ARM_MOTOR_COUNT) {
        return (int8_t)(motor_id - 1U);
    }
    return -1;
}

static uint16_t arm_motor_id_from_index(uint8_t index)
{
    return g_arm_motor_cfg[index].can_id;
}

static float arm_read_f32_le(const uint8_t *p)
{
    float out = 0.0f;
    memcpy(&out, p, sizeof(float));
    return out;
}

static void arm_send_frame(uint8_t cmd, const uint8_t *payload, uint8_t length)
{
    uint8_t cs = (uint8_t)(length + cmd);
    uint8_t i;

    // Reply only on USART1 (Pi connects via CH9102 USB bridge -> PA9/PA10).
    // Sending on USART2/3 with no device attached causes TC-flag spin-lock.
    while ((USART1->SR & 0X40) == 0) {}
    USART1->DR = PROTOCOL_HEADER;

    while ((USART1->SR & 0X40) == 0) {}
    USART1->DR = length;

    while ((USART1->SR & 0X40) == 0) {}
    USART1->DR = cmd;

    for (i = 0U; i < length; i++) {
        cs = (uint8_t)(cs + payload[i]);
        while ((USART1->SR & 0X40) == 0) {}
        USART1->DR = payload[i];
    }

    while ((USART1->SR & 0X40) == 0) {}
    USART1->DR = cs;
}

static uint8_t arm_motor_is_vel_mode(uint8_t index)
{
    if (index >= ARM_MOTOR_COUNT) return 0U;
    return (g_arm_motor_cfg[index].type == DM_MULTI_TYPE_DM3510) ? 1U : 0U;
}

static void arm_apply_motor_command(uint8_t index)
{
    ArmMotorCmd_t *cmd;
    uint16_t can_id;
    float p_des;
    float v_des;
    float t_cmd;
    DM_MultiFeedback_t fb;
    uint8_t has_fb;
    uint8_t vel_mode;

    if (index >= ARM_MOTOR_COUNT) {
        return;
    }

    cmd = &g_arm_cmd[index];
    can_id = arm_motor_id_from_index(index);
    vel_mode = arm_motor_is_vel_mode(index);

    if (!cmd->enabled) {
        if (g_arm_applied_enable[index]) {
            if (!vel_mode) {
                DM_Multi_SetMit(can_id, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f);
            } else {
                DM_Multi_SetVel(can_id, 0.0f);
            }
            DM_Multi_Disable(can_id);
            g_arm_applied_enable[index] = 0U;
        }
        return;
    }

    if (!g_arm_applied_enable[index]) {
        DM_Multi_ClearError(can_id);
        DM_Multi_Disable(can_id);
        delay_ms(5);
        if (vel_mode) {
            DM_Multi_SetControlModeVel(can_id);
        } else {
            DM_Multi_SetControlModeMit(can_id);
        }
        delay_ms(5);
        DM_Multi_EnableCompat(can_id);
        g_arm_applied_enable[index] = 1U;
        if (DM_Multi_GetFeedbackByIndex(index, &fb)) {
            cmd->p_des = fb.pos_rad;
            cmd->target_position = fb.pos_rad;
        }
    }

    if (vel_mode) {
        /* VEL mode: send float velocity directly */
        DM_Multi_SetVel(can_id, cmd->target_velocity);
        return;
    }

    /* MIT mode path (DM4310 etc.) */
    p_des = cmd->p_des;
    v_des = cmd->v_des;
    if (cmd->control_mode != ARM_MODE_MIT) {
        p_des = cmd->target_position;
        v_des = cmd->target_velocity;
    }

    has_fb = DM_Multi_GetFeedbackByIndex(index, &fb);
    t_cmd = cmd->t_ff;
    if (has_fb) {
        t_cmd += arm_gravity_ff(index, &fb);
    }
    DM_Multi_SetMit(can_id, p_des, v_des, cmd->kp, cmd->kd, t_cmd);
}

static void arm_set_velocity_percent(uint8_t motor_id, float pct, uint8_t dir)
{
    int8_t idx = arm_index_from_motor_id(motor_id);
    float v_cmd;
    DM_MultiFeedback_t vel_fb;

    if (idx < 0) {
        return;
    }

    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;

    v_cmd = (pct * g_arm_full_scale_vel[idx]) / 100.0f;
    if (dir == 0U) {
        v_cmd = -v_cmd;
    } else if (dir == DIR_STOP) {
        v_cmd = 0.0f;
    }

    /* NOTE: enabled state NOT changed here – managed separately. */
    g_arm_cmd[idx].control_mode = ARM_MODE_MIT;
    g_arm_cmd[idx].motor_group = ARM_GROUP_A;
    g_arm_cmd[idx].target_velocity = v_cmd;

    /* Snapshot on start, latch on stop – same as arm_set_velocity_mrad. */
    if (v_cmd != 0.0f && g_arm_cmd[idx].v_des == 0.0f) {
        if (DM_Multi_GetFeedbackByIndex((uint8_t)idx, &vel_fb)) {
            g_arm_cmd[idx].p_des = vel_fb.pos_rad;
            g_arm_cmd[idx].target_position = vel_fb.pos_rad;
        }
    }
    if (v_cmd == 0.0f && g_arm_cmd[idx].v_des != 0.0f) {
        if (DM_Multi_GetFeedbackByIndex((uint8_t)idx, &vel_fb)) {
            g_arm_cmd[idx].p_des = vel_fb.pos_rad;
            g_arm_cmd[idx].target_position = vel_fb.pos_rad;
        }
        g_arm_cmd[idx].v_des = 0.0f;
    }
    g_status_focus_index = (uint8_t)idx;
}

static void arm_set_velocity_mrad(uint8_t motor_id, int16_t vel_mrad_s)
{
    int8_t idx = arm_index_from_motor_id(motor_id);
    DM_MultiFeedback_t vel_fb;
    if (idx < 0) {
        return;
    }

    /* NOTE: enabled state is NOT changed here – managed by enable command
       or gateway control. This allows vel=0 to mean "hold" not "disable". */

    g_arm_cmd[idx].control_mode = ARM_MODE_MIT;
    g_arm_cmd[idx].motor_group = ARM_GROUP_A;
    g_arm_cmd[idx].target_velocity = ((float)vel_mrad_s) / 1000.0f;

    /* When transitioning from standstill to motion, snapshot current
       position as the ramp starting point. */
    if (vel_mrad_s != 0 && g_arm_cmd[idx].v_des == 0.0f) {
        if (DM_Multi_GetFeedbackByIndex((uint8_t)idx, &vel_fb)) {
            g_arm_cmd[idx].p_des = vel_fb.pos_rad;
            g_arm_cmd[idx].target_position = vel_fb.pos_rad;
        }
    }
    /* When velocity becomes zero, latch current position for position hold. */
    if (vel_mrad_s == 0 && g_arm_cmd[idx].v_des != 0.0f) {
        if (DM_Multi_GetFeedbackByIndex((uint8_t)idx, &vel_fb)) {
            g_arm_cmd[idx].p_des = vel_fb.pos_rad;
            g_arm_cmd[idx].target_position = vel_fb.pos_rad;
        }
        g_arm_cmd[idx].v_des = 0.0f;
    }
    g_status_focus_index = (uint8_t)idx;
    /* Do NOT call arm_apply_motor_command here – the periodic tick handles
       smooth ramping of p_des and sending MIT frames every 10 ms. */
}

static void arm_set_enable(uint8_t motor_id, uint8_t enable)
{
    uint8_t i;
    if (motor_id == 0xFFU) {
        for (i = 0U; i < ARM_MOTOR_COUNT; i++) {
            g_arm_cmd[i].enabled = enable ? 1U : 0U;
            if (!enable) {
                g_arm_cmd[i].v_des = 0.0f;
                g_arm_cmd[i].p_des = 0.0f;
                g_arm_cmd[i].t_ff = 0.0f;
                g_arm_cmd[i].target_velocity = 0.0f;
            }
            arm_apply_motor_command(i);
        }
        return;
    }

    {
        int8_t idx = arm_index_from_motor_id(motor_id);
        if (idx < 0) {
            return;
        }
        g_arm_cmd[idx].enabled = enable ? 1U : 0U;
        if (!enable) {
            g_arm_cmd[idx].v_des = 0.0f;
            g_arm_cmd[idx].p_des = 0.0f;
            g_arm_cmd[idx].t_ff = 0.0f;
            g_arm_cmd[idx].target_velocity = 0.0f;
        }
        g_status_focus_index = (uint8_t)idx;
        arm_apply_motor_command((uint8_t)idx);
    }
}

static void arm_stop_all(void)
{
    uint8_t i;
    for (i = 0U; i < ARM_MOTOR_COUNT; i++) {
        g_arm_cmd[i].enabled = 0U;
        g_arm_cmd[i].v_des = 0.0f;
        g_arm_cmd[i].p_des = 0.0f;
        g_arm_cmd[i].t_ff = 0.0f;
        g_arm_cmd[i].target_velocity = 0.0f;
        arm_apply_motor_command(i);
    }
}

static void arm_handle_scan_ids(void)
{
    uint8_t payload[1U + 31U];
    uint8_t ids[31U];
    uint8_t i;
    uint8_t count;

    DM_Multi_ClearSeenIds();
    for (i = 1U; i < 32U; i++) {
        DM_Multi_RequestStatus(i);
        delay_ms(2);
    }

    count = DM_Multi_GetSeenIds(ids, 31U);
    payload[0] = count;
    for (i = 0U; i < count; i++) {
        payload[1U + i] = ids[i];
    }

    arm_send_frame(CMD_DM_SCAN_IDS_REPLY, payload, (uint8_t)(count + 1U));
}

static void arm_handle_set_id(uint8_t old_id, uint8_t new_id)
{
    uint8_t payload[4];
    uint8_t ok;

    DM_Multi_ClearLastRegAck();
    ok = DM_Multi_SetMotorId(old_id, new_id, 1U);
    delay_ms(30);

    payload[0] = old_id;
    payload[1] = new_id;
    payload[2] = ok ? 1U : 0U;
    payload[3] = 0U;
    arm_send_frame(CMD_DM_SET_ID_REPLY, payload, sizeof(payload));
}

static void arm_apply_gateway_command(uint8_t control_mode,
                                      uint8_t motor_group,
                                      uint8_t motor_id,
                                      uint8_t enable,
                                      float p_des,
                                      float v_des,
                                      float kp,
                                      float kd,
                                      float t_ff,
                                      float target_position,
                                      float target_velocity)
{
    int8_t idx;
    uint8_t i;

    if (motor_id == 0xFFU) {
        for (i = 0U; i < ARM_MOTOR_COUNT; i++) {
            g_arm_cmd[i].control_mode = control_mode;
            g_arm_cmd[i].motor_group = motor_group;
            g_arm_cmd[i].enabled = enable ? 1U : 0U;
            g_arm_cmd[i].p_des = p_des;
            g_arm_cmd[i].v_des = v_des;
            g_arm_cmd[i].kp = kp;
            g_arm_cmd[i].kd = kd;
            g_arm_cmd[i].t_ff = t_ff;
            g_arm_cmd[i].target_position = target_position;
            g_arm_cmd[i].target_velocity = target_velocity;
            arm_apply_motor_command(i);
        }
        g_status_focus_index = 0U;
        return;
    }

    idx = arm_index_from_motor_id(motor_id);
    if (idx < 0) {
        return;
    }

    g_arm_cmd[idx].control_mode = control_mode;
    g_arm_cmd[idx].motor_group = motor_group;
    g_arm_cmd[idx].enabled = enable ? 1U : 0U;
    g_arm_cmd[idx].p_des = p_des;
    g_arm_cmd[idx].v_des = v_des;
    g_arm_cmd[idx].kp = kp;
    g_arm_cmd[idx].kd = kd;
    g_arm_cmd[idx].t_ff = t_ff;
    g_arm_cmd[idx].target_position = target_position;
    g_arm_cmd[idx].target_velocity = target_velocity;
    g_status_focus_index = (uint8_t)idx;
    arm_apply_motor_command((uint8_t)idx);
}

static void arm_parse_gateway_frame(const uint8_t *frame)
{
    uint8_t i;
    uint8_t cs = 0U;
    uint8_t cmd;

    if (frame[0] != 0xAAU || frame[1] != 0x55U) {
        return;
    }

    for (i = 2U; i < (ARM_GATEWAY_FRAME_LEN - 1U); i++) {
        cs = (uint8_t)(cs + frame[i]);
    }
    if (cs != frame[ARM_GATEWAY_FRAME_LEN - 1U]) {
        return;
    }

    cmd = frame[3];
    if (cmd != CMD_GATEWAY_CONTROL) {
        return;
    }

    arm_apply_gateway_command(
        frame[4],
        frame[5],
        frame[6],
        frame[7],
        arm_read_f32_le(&frame[8]),
        arm_read_f32_le(&frame[12]),
        arm_read_f32_le(&frame[16]),
        arm_read_f32_le(&frame[20]),
        arm_read_f32_le(&frame[24]),
        arm_read_f32_le(&frame[28]),
        arm_read_f32_le(&frame[32])
    );
}

static void arm_poll_gateway_bytes(uint8_t byte)
{
    if (g_gateway_index == 0U) {
        if (byte == 0xAAU) {
            g_gateway_buf[0] = byte;
            g_gateway_index = 1U;
        }
        return;
    }

    if (g_gateway_index == 1U) {
        if (byte == 0x55U) {
            g_gateway_buf[1] = byte;
            g_gateway_index = 2U;
        } else if (byte == 0xAAU) {
            g_gateway_buf[0] = byte;
            g_gateway_index = 1U;
        } else {
            g_gateway_index = 0U;
        }
        return;
    }

    g_gateway_buf[g_gateway_index++] = byte;
    if (g_gateway_index >= ARM_GATEWAY_FRAME_LEN) {
        arm_parse_gateway_frame(g_gateway_buf);
        g_gateway_index = 0U;
    }
}

void Protocol_Init(void)
{
    rx_state = STATE_WAIT_HEADER;
    rx_data_index = 0;
    calc_checksum = 0;

    {
        uint8_t i;

        DM_Multi_Init(g_arm_motor_cfg, ARM_MOTOR_COUNT);
        g_status_focus_index = 0U;
        g_gateway_index = 0U;
        g_arm_service_elapsed_ms = 0U;
        g_arm_status_elapsed_ms = 0U;
        g_arm_comm_watchdog_ms = 0U;
        g_arm_watchdog_tripped = 0U;

        for (i = 0U; i < ARM_MOTOR_COUNT; i++) {
            g_arm_cmd[i].enabled = 0U;
            g_arm_cmd[i].control_mode = ARM_MODE_MIT;
            g_arm_cmd[i].motor_group = ARM_GROUP_A;
            g_arm_cmd[i].p_des = 0.0f;
            g_arm_cmd[i].v_des = 0.0f;
            g_arm_cmd[i].t_ff = 0.0f;
            g_arm_cmd[i].target_position = 0.0f;
            g_arm_cmd[i].target_velocity = 0.0f;
            g_arm_applied_enable[i] = 0U;
            g_arm_grav[i].enabled = 0U;
            g_arm_grav[i].zero_initialized = 0U;
            g_arm_grav[i].zero_pos_rad = 0.0f;
            g_arm_grav[i].kg = 0.0f;
            g_arm_grav[i].bias_rad = 0.0f;
            g_arm_grav[i].viscous = 0.0f;
            g_arm_grav[i].coulomb = 0.0f;
            /* Per-motor KP/KD defaults (from datasheet peak-torque budget):
             *   index 0 = DM4310  (id 1): arm base,       peak 7 N.m, 10:1, KP=12
             *   index 1 = DMH3510 (id 2): arm joint1,     peak 0.45 N.m, direct, KP=3
             *   index 2 = DMH3510 (id 3): support leg L,  KP=3
             *   index 3 = DMH3510 (id 4): support leg R,  KP=3 */
            if (i == 0U) {
                /* DM4310 base joint */
                g_arm_cmd[i].kp = 12.0f;
                g_arm_cmd[i].kd =  1.5f;
            } else {
                /* DMH3510 – arm joint or support leg, direct drive, keep KP low */
                g_arm_cmd[i].kp =  3.0f;
                g_arm_cmd[i].kd =  0.3f;
            }

            DM_Multi_ClearError(arm_motor_id_from_index(i));
            DM_Multi_SetControlModeMit(arm_motor_id_from_index(i));
            DM_Multi_Disable(arm_motor_id_from_index(i));
        }

        // Initial gravity compensation defaults (tuned online later).
        g_arm_grav[0].enabled = 1U; g_arm_grav[0].kg = 0.55f; g_arm_grav[0].coulomb = 0.03f; // ID1 DM4310 arm base
        g_arm_grav[1].enabled = 1U; g_arm_grav[1].kg = 0.40f; g_arm_grav[1].coulomb = 0.03f; // ID2 DMH3510 arm joint1
    }
}

void Protocol_TickMs(uint16_t elapsed_ms)
{
    uint8_t i;

    g_arm_service_elapsed_ms += elapsed_ms;
    g_arm_status_elapsed_ms  += elapsed_ms;
    g_arm_beacon_elapsed_ms  += elapsed_ms;

    if (g_arm_service_elapsed_ms >= ARM_SERVICE_PERIOD_MS) {
        g_arm_service_elapsed_ms = 0U;
        for (i = 0U; i < ARM_MOTOR_COUNT; i++) {
            if (g_arm_cmd[i].enabled) {
                /* ── Trajectory ramp generator ──
                 * When target_velocity != 0, smoothly ramp p_des so that:
                 *   Kp*(p_des − p_act) tracks a small error (≈ one tick)
                 *   Kd*(v_des − v_act) helps maintain speed, not brakes
                 * When target_velocity == 0, hold: p_des stays, v_des = 0. */
                if (g_arm_cmd[i].target_velocity != 0.0f) {
                    float dt_s = (float)ARM_SERVICE_PERIOD_MS / 1000.0f;
                    g_arm_cmd[i].p_des += g_arm_cmd[i].target_velocity * dt_s;
                    g_arm_cmd[i].v_des = g_arm_cmd[i].target_velocity;
                }
                /* else: v_des and p_des already set by the latch logic */
                arm_apply_motor_command(i);
            }
        }
    }

    if (g_arm_status_elapsed_ms >= ARM_STATUS_POLL_PERIOD_MS) {
        g_arm_status_elapsed_ms = 0U;
        DM_Multi_RequestStatus(arm_motor_id_from_index(g_status_focus_index));
    }

    // Broadcast a status frame once per second so the host can detect
    // the link without sending a command first.
    if (g_arm_beacon_elapsed_ms >= 1000U) {
        g_arm_beacon_elapsed_ms = 0U;
        Protocol_SendStatus();
    }

    /* Communication watchdog: if no valid frame received within
     * ARM_COMM_WATCHDOG_MS, disable all motors for safety.
     * This protects against SSH disconnect / serial cable unplug. */
    g_arm_comm_watchdog_ms += elapsed_ms;
    if (g_arm_comm_watchdog_ms >= ARM_COMM_WATCHDOG_MS && !g_arm_watchdog_tripped) {
        g_arm_watchdog_tripped = 1U;
        arm_stop_all();
    }
}

/**
 * @brief 在串口中断中按字节解析协议
 * Legacy frame: [HEAD=0xAA] [LEN] [CMD] [DATA...] [CHECKSUM]
 */
void Protocol_ParseByte(uint8_t byte)
{
    arm_poll_gateway_bytes(byte);

    switch (rx_state) {
        case STATE_WAIT_HEADER:
            if (byte == PROTOCOL_HEADER) {
                current_frame.header = byte;
                rx_state = STATE_WAIT_LENGTH;
            }
            break;

        case STATE_WAIT_LENGTH:
            if (byte <= MAX_PAYLOAD_LEN) {
                current_frame.length = byte;
                calc_checksum = byte;
                rx_state = STATE_WAIT_CMD;
            } else {
                rx_state = STATE_WAIT_HEADER;
            }
            break;

        case STATE_WAIT_CMD:
            current_frame.cmd = byte;
            calc_checksum = (uint8_t)(calc_checksum + byte);
            rx_data_index = 0;
            if (current_frame.length > 0U) {
                rx_state = STATE_WAIT_DATA;
            } else {
                rx_state = STATE_WAIT_CHECKSUM;
            }
            break;

        case STATE_WAIT_DATA:
            current_frame.payload[rx_data_index++] = byte;
            calc_checksum = (uint8_t)(calc_checksum + byte);
            if (rx_data_index >= current_frame.length) {
                rx_state = STATE_WAIT_CHECKSUM;
            }
            break;

        case STATE_WAIT_CHECKSUM:
            current_frame.checksum = byte;
            if (current_frame.checksum == calc_checksum) {
                Protocol_HandleFrame(&current_frame);
            }
            rx_state = STATE_WAIT_HEADER;
            break;

        default:
            rx_state = STATE_WAIT_HEADER;
            break;
    }
}

void Protocol_HandleFrame(ProtocolFrame_t *frame)
{
    /* Any valid frame resets the communication watchdog. */
    g_arm_comm_watchdog_ms = 0U;
    g_arm_watchdog_tripped = 0U;

    switch (frame->cmd) {
        case CMD_SET_MOTOR_SPEED:
            if (frame->length >= 3U) {
                uint8_t motor_id = frame->payload[0];
                float speed = (float)frame->payload[1];
                uint8_t dir = frame->payload[2];

                if (motor_id == 0xFFU) {
                    uint8_t i;
                    for (i = 1U; i <= ARM_MOTOR_COUNT; i++) {
                        arm_set_velocity_percent(i, speed, dir);
                    }
                } else {
                    arm_set_velocity_percent(motor_id, speed, dir);
                }

                if (dir != DIR_STOP && speed > 0.0f) {
                    LED0 = 0;
                } else {
                    LED0 = 1;
                }
            }
            break;

        case CMD_SET_MOTOR_VEL_MRAD:
            if (frame->length >= 3U) {
                uint8_t motor_id = frame->payload[0];
                int16_t vel_mrad_i16 = (int16_t)((((uint16_t)frame->payload[2]) << 8) |
                                                 ((uint16_t)frame->payload[1]));

                if (motor_id == 0xFFU) {
                    uint8_t i;
                    for (i = 1U; i <= ARM_MOTOR_COUNT; i++) {
                        arm_set_velocity_mrad(i, vel_mrad_i16);
                    }
                } else {
                    arm_set_velocity_mrad(motor_id, vel_mrad_i16);
                }

                if (vel_mrad_i16 != 0) {
                    LED0 = 0;
                } else {
                    LED0 = 1;
                }
            }
            break;

        case CMD_STOP_ALL:
            arm_stop_all();
            LED0 = 1;
            break;

        case CMD_GET_STATUS:
            Protocol_SendStatus();
            break;

        case CMD_SET_ENABLE:
            if (frame->length >= 2U) {
                uint8_t motor_id = frame->payload[0];
                uint8_t enable = frame->payload[1];
                arm_set_enable(motor_id, enable ? 1U : 0U);
                if (!enable) {
                    LED0 = 1;
                }
            }
            break;

        case CMD_DM_SCAN_IDS:
            arm_handle_scan_ids();
            break;

        case CMD_DM_SET_ID:
            if (frame->length >= 2U) {
                arm_handle_set_id(frame->payload[0], frame->payload[1]);
            }
            break;

        case CMD_DM_SAVE_ZERO:
            if (frame->length >= 1U) {
                uint8_t mid = frame->payload[0];
                if (mid == 0xFFU) {
                    uint8_t i;
                    for (i = 0U; i < ARM_MOTOR_COUNT; i++) {
                        DM_Multi_SaveZero(arm_motor_id_from_index(i));
                    }
                } else {
                    DM_Multi_SaveZero(mid);
                }
            }
            break;

        case CMD_SET_GRAVITY_COMP:
            // payload: [motor_id][enable][kg(float)][bias(float)][viscous(float)][coulomb(float)]
            if (frame->length >= 18U) {
                uint8_t mid = frame->payload[0];
                uint8_t en = frame->payload[1];
                float kg = arm_read_f32_le(&frame->payload[2]);
                float bias = arm_read_f32_le(&frame->payload[6]);
                float viscous = arm_read_f32_le(&frame->payload[10]);
                float coulomb = arm_read_f32_le(&frame->payload[14]);
                if (mid == 0xFFU) {
                    uint8_t i;
                    for (i = 0U; i < ARM_MOTOR_COUNT; i++) {
                        g_arm_grav[i].enabled = en ? 1U : 0U;
                        g_arm_grav[i].kg = kg;
                        g_arm_grav[i].bias_rad = bias;
                        g_arm_grav[i].viscous = viscous;
                        g_arm_grav[i].coulomb = coulomb;
                    }
                } else {
                    int8_t idx = arm_index_from_motor_id(mid);
                    if (idx >= 0) {
                        g_arm_grav[idx].enabled = en ? 1U : 0U;
                        g_arm_grav[idx].kg = kg;
                        g_arm_grav[idx].bias_rad = bias;
                        g_arm_grav[idx].viscous = viscous;
                        g_arm_grav[idx].coulomb = coulomb;
                    }
                }
            }
            break;

        case CMD_REZERO_GRAVITY:
            // payload: [motor_id]  (0xFF = all)
            if (frame->length >= 1U) {
                uint8_t mid = frame->payload[0];
                if (mid == 0xFFU) {
                    uint8_t i;
                    for (i = 0U; i < ARM_MOTOR_COUNT; i++) {
                        DM_MultiFeedback_t fb;
                        if (DM_Multi_GetFeedbackByIndex(i, &fb)) {
                            g_arm_grav[i].zero_pos_rad = fb.pos_rad;
                            g_arm_grav[i].zero_initialized = 1U;
                        } else {
                            g_arm_grav[i].zero_initialized = 0U;
                        }
                    }
                } else {
                    int8_t idx = arm_index_from_motor_id(mid);
                    DM_MultiFeedback_t fb;
                    if (idx >= 0) {
                        if (DM_Multi_GetFeedbackByIndex((uint8_t)idx, &fb)) {
                            g_arm_grav[idx].zero_pos_rad = fb.pos_rad;
                            g_arm_grav[idx].zero_initialized = 1U;
                        } else {
                            g_arm_grav[idx].zero_initialized = 0U;
                        }
                    }
                }
            }
            break;

        case CMD_DM_REG_READ:
            // payload: [motor_id(1)] [register_id(1)]
            if (frame->length >= 2U) {
                uint8_t mid = frame->payload[0];
                uint8_t rid = frame->payload[1];
                uint8_t rpl[7];
                DM_MultiRegAck_t ack;
                uint16_t wait;
                DM_Multi_ClearLastRegAck();
                DM_Multi_ReadRegister(mid, rid);
                for (wait = 0U; wait < 30U; wait++) {
                    delay_ms(1);
                    if (DM_Multi_GetLastRegAck(&ack)) break;
                }
                rpl[0] = mid;
                rpl[1] = rid;
                if (DM_Multi_GetLastRegAck(&ack) && ack.valid) {
                    rpl[2] = 1U; // success
                    memcpy(&rpl[3], &ack.value_u32, 4U);
                } else {
                    rpl[2] = 0U; // fail
                    memset(&rpl[3], 0, 4U);
                }
                arm_send_frame(CMD_DM_REG_READ_REPLY, rpl, 7U);
            }
            break;

        case CMD_DM_REG_WRITE:
            // payload: [motor_id(1)] [register_id(1)] [value_le(4)]
            if (frame->length >= 6U) {
                uint8_t mid = frame->payload[0];
                uint8_t rid = frame->payload[1];
                uint32_t val;
                uint8_t rpl[7];
                DM_MultiRegAck_t ack;
                uint16_t wait;
                memcpy(&val, &frame->payload[2], 4U);
                DM_Multi_ClearLastRegAck();
                DM_Multi_WriteRegisterU32(mid, rid, val);
                for (wait = 0U; wait < 30U; wait++) {
                    delay_ms(1);
                    if (DM_Multi_GetLastRegAck(&ack)) break;
                }
                rpl[0] = mid;
                rpl[1] = rid;
                if (DM_Multi_GetLastRegAck(&ack) && ack.valid) {
                    rpl[2] = 1U;
                    memcpy(&rpl[3], &ack.value_u32, 4U);
                } else {
                    rpl[2] = 0U;
                    memset(&rpl[3], 0, 4U);
                }
                arm_send_frame(CMD_DM_REG_WRITE_REPLY, rpl, 7U);
            }
            break;

        case CMD_DM_REG_SAVE:
            // payload: [motor_id(1)]  -- 写寄存器 0x01 = 1 保存参数到Flash
            if (frame->length >= 1U) {
                uint8_t mid = frame->payload[0];
                DM_Multi_WriteRegisterU32(mid, 0x01U, 1UL);
                delay_ms(100);
            }
            break;

        case CMD_DM_RAW_CAN:
            // payload: [std_id_lo(1)] [std_id_hi(1)] [d0..d7(8)]
            if (frame->length >= 10U) {
                uint16_t sid = (uint16_t)(((uint16_t)frame->payload[1] << 8) | frame->payload[0]);
                DM_Multi_SendRawCAN(sid, &frame->payload[2]);
            }
            break;

        default:
            break;
    }
}

void Protocol_SendStatus(void)
{
    uint8_t payload[ARM_STATUS_PAYLOAD_LEN];
    DM_MultiFeedback_t fb;
    ArmMotorCmd_t *cmd;
    uint8_t has_fb;
    uint8_t status;

    if (g_status_focus_index >= ARM_MOTOR_COUNT) {
        g_status_focus_index = 0U;
    }

    cmd = &g_arm_cmd[g_status_focus_index];
    has_fb = DM_Multi_GetFeedbackByIndex(g_status_focus_index, &fb);

    payload[0] = cmd->motor_group;
    payload[1] = cmd->control_mode;
    payload[2] = (uint8_t)arm_motor_id_from_index(g_status_focus_index);
    payload[3] = cmd->enabled ? 1U : 0U;

    if (has_fb) {
        memcpy(&payload[4], &fb.pos_rad, sizeof(float));
        memcpy(&payload[8], &fb.vel_rad_s, sizeof(float));
        memcpy(&payload[12], &fb.tau_nm, sizeof(float));
        status = fb.err;
    } else {
        float zero = 0.0f;
        memcpy(&payload[4], &zero, sizeof(float));
        memcpy(&payload[8], &zero, sizeof(float));
        memcpy(&payload[12], &zero, sizeof(float));
        status = 1U;
    }

    payload[16] = status;
    arm_send_frame(CMD_STATUS_REPLY, payload, ARM_STATUS_PAYLOAD_LEN);
}
