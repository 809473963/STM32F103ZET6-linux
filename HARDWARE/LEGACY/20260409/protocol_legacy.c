#include "protocol.h"
#if defined(MOTOR86_APP)
#include "motor86.h"
#elif !defined(ENCODER_MOTOR_APP)
#include "motor.h"
#else
#include "dm4310_can.h"
#include "delay.h"
#endif
#include "usart.h" // 使用现有的 USART 发送函数
#include "led.h"   // 闪烁 LED 指示收发状态

static ProtocolState_t rx_state = STATE_WAIT_HEADER;
static ProtocolFrame_t current_frame;
static uint8_t rx_data_index = 0;
static uint8_t calc_checksum = 0;

#ifndef DIR_STOP
#define DIR_STOP 2
#endif

#ifdef ENCODER_MOTOR_APP
// DM4310 velocity command mapping: 0~100% -> 0~14rad/s
#define DM4310_VEL_MAX_MRAD_S 14000
#define DM4310_KD_CMD 3.0f
#define DM4310_TFF_BOOST_MNM 4800
#define DM4310_VEL_RAMP_UP_MRAD_PER_MS 10
#define DM4310_VEL_RAMP_DOWN_MRAD_PER_MS 16

// Idle hold tuning for loaded arm.
#define DM4310_HOLD_KP_CMD 12.0f
// dm4310_can.c clamps kd to <=5. Keep this below 5 to avoid hidden truncation.
#define DM4310_HOLD_KD_CMD 4.8f
#define DM4310_HOLD_TRIM_KI_NM_PER_RAD_S 2.8f
#define DM4310_HOLD_TRIM_MAX_NM 1.2f
#define DM4310_GRAV_FF_AMP_NM 2.4f
#define DM4310_GRAV_FF_OFFSET_RAD 0.0f
#define DM4310_HOLD_TFF_MAX_NM 4.0f
#define DM4310_HOLD_CAPTURE_DEADBAND_RAD 0.015f
#define DM4310_HOLD_RESET_ERR_RAD 0.45f
#define DM4310_HOLD_TFF_STEP_NM 0.06f

#define DM4310_FB_POS_MIN_RAD (-12.5f)
#define DM4310_FB_POS_MAX_RAD (12.5f)
#define DM4310_FB_POS_SPAN_RAD (25.0f)
#define DM4310_PI 3.1415926f
#define DM4310_TWO_PI 6.2831852f

static volatile int16_t s_dm_target_vel_mrad_s = 0;
static volatile int16_t s_dm_ramped_vel_mrad_s = 0;
static volatile uint8_t s_dm_req_enable = 0;
static volatile uint8_t s_dm_req_disable = 0;
static volatile uint16_t s_dm_cmd_age_ms = 0;

static uint8_t s_dm_enabled = 0;
static uint16_t s_dm_tx_period_ms = 0;
static uint8_t s_dm_enable_seq_busy = 0;
static uint8_t s_dm_hold_active = 0;
static float s_dm_hold_pos_rad = 0.0f;
static float s_dm_hold_trim_nm = 0.0f;
static float s_dm_hold_tff_nm = 0.0f;
static uint8_t s_dm_pos_unwrap_init = 0;
static float s_dm_pos_raw_prev = 0.0f;
static float s_dm_pos_unwrapped = 0.0f;

static float clamp_f32(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float slew_f32(float current, float target, float step)
{
    if (target > current + step) return current + step;
    if (target < current - step) return current - step;
    return target;
}

static float abs_f32(float x)
{
    return (x < 0.0f) ? -x : x;
}

// Small cosine approximation to avoid libm dependency in bare-metal build.
static float cos_approx_f32(float x)
{
    float x2;

    while (x > DM4310_PI) x -= DM4310_TWO_PI;
    while (x < -DM4310_PI) x += DM4310_TWO_PI;

    x2 = x * x;
    return 1.0f - x2 * (0.5f - x2 * (0.04166667f - x2 * 0.00138889f));
}

static float unwrap_pos_f32(float raw_pos)
{
    float delta;
    float half_span = DM4310_FB_POS_SPAN_RAD * 0.5f;

    if (!s_dm_pos_unwrap_init) {
        s_dm_pos_unwrap_init = 1;
        s_dm_pos_raw_prev = raw_pos;
        s_dm_pos_unwrapped = raw_pos;
        return s_dm_pos_unwrapped;
    }

    delta = raw_pos - s_dm_pos_raw_prev;
    if (delta > half_span) {
        delta -= DM4310_FB_POS_SPAN_RAD;
    } else if (delta < -half_span) {
        delta += DM4310_FB_POS_SPAN_RAD;
    }

    s_dm_pos_unwrapped += delta;
    s_dm_pos_raw_prev = raw_pos;
    return s_dm_pos_unwrapped;
}

static float wrap_pos_to_driver_range(float pos_rad)
{
    while (pos_rad > DM4310_FB_POS_MAX_RAD) pos_rad -= DM4310_FB_POS_SPAN_RAD;
    while (pos_rad < DM4310_FB_POS_MIN_RAD) pos_rad += DM4310_FB_POS_SPAN_RAD;
    return pos_rad;
}

static int16_t clamp_i16(int32_t x, int16_t lo, int16_t hi)
{
    if (x < (int32_t)lo) return lo;
    if (x > (int32_t)hi) return hi;
    return (int16_t)x;
}

static int16_t ramp_velocity_i16(int16_t current, int16_t target, uint16_t elapsed_ms)
{
    int32_t cur = current;
    int32_t tgt = target;
    int32_t step_up = (int32_t)DM4310_VEL_RAMP_UP_MRAD_PER_MS * (int32_t)elapsed_ms;
    int32_t step_down = (int32_t)DM4310_VEL_RAMP_DOWN_MRAD_PER_MS * (int32_t)elapsed_ms;
    int32_t step;

    if (step_up < 1) step_up = 1;
    if (step_down < 1) step_down = 1;

    step = step_up;
    if (((cur > 0) && (tgt < 0)) || ((cur < 0) && (tgt > 0)) ||
        (abs_f32((float)tgt) < abs_f32((float)cur))) {
        step = step_down;
    }

    if (cur < tgt) {
        cur += step;
        if (cur > tgt) cur = tgt;
    } else if (cur > tgt) {
        cur -= step;
        if (cur < tgt) cur = tgt;
    }

    return (int16_t)cur;
}
#endif

void Protocol_Init(void)
{
    rx_state = STATE_WAIT_HEADER;
#ifdef ENCODER_MOTOR_APP
    s_dm_target_vel_mrad_s = 0;
    s_dm_ramped_vel_mrad_s = 0;
    // Requirement: enable once when firmware starts.
    s_dm_req_enable = 1;
    s_dm_req_disable = 0;
    s_dm_cmd_age_ms = 0;
    s_dm_enabled = 0;
    s_dm_tx_period_ms = 0;
    s_dm_enable_seq_busy = 0;
    s_dm_hold_active = 0;
    s_dm_hold_pos_rad = 0.0f;
    s_dm_hold_trim_nm = 0.0f;
    s_dm_hold_tff_nm = 0.0f;
    s_dm_pos_unwrap_init = 0;
    s_dm_pos_raw_prev = 0.0f;
    s_dm_pos_unwrapped = 0.0f;
#endif
}

void Protocol_TickMs(uint16_t elapsed_ms)
{
#ifdef ENCODER_MOTOR_APP
    int16_t vel_mrad;
    float pos_err_rad;
    float hold_cmd_pos_rad;
    float grav_tff_nm;
    float trim_target_nm;
    float hold_tff_target_nm;
    float vel_rad_s;
    int16_t t_ff_mnm;
    float t_ff_nm;
    float fb_pos_unwrapped;
    DM4310_Feedback_t fb;
    uint8_t fb_ok;

    s_dm_cmd_age_ms = (uint16_t)(s_dm_cmd_age_ms + elapsed_ms);
    s_dm_tx_period_ms = (uint16_t)(s_dm_tx_period_ms + elapsed_ms);

    // Execute heavy enable/disable routines in main loop, not in IRQ context.
    if (s_dm_req_disable) {
        s_dm_req_disable = 0;
        s_dm_target_vel_mrad_s = 0;
        s_dm_ramped_vel_mrad_s = 0;
        DM4310_SetMitCommand(0.0f, 0.0f, 0.0f, 1.0f, 0.0f);
        DM4310_SendMitNow();
        DM4310_Disable();
        s_dm_enabled = 0;
        s_dm_enable_seq_busy = 0;
        s_dm_hold_active = 0;
        s_dm_hold_trim_nm = 0.0f;
        s_dm_hold_tff_nm = 0.0f;
        s_dm_pos_unwrap_init = 0;
        printf("DM4310 DISABLED\r\n");
        LED0 = 1;
    }

    if (s_dm_req_enable && !s_dm_enable_seq_busy && !s_dm_enabled) {
        s_dm_enable_seq_busy = 1;
        s_dm_req_enable = 0;
        DM4310_ClearError();
        DM4310_EnterMitMode();
        delay_ms(20);
        DM4310_EnableCompat();
        s_dm_enabled = 1;
        s_dm_enable_seq_busy = 0;
        printf("DM4310 ENABLED\r\n");
    }

    // Link-loss safety: if no keyboard command for a while, force stop velocity.
    if (s_dm_cmd_age_ms > 400U) {
        s_dm_target_vel_mrad_s = 0;
    }

    s_dm_ramped_vel_mrad_s = ramp_velocity_i16(s_dm_ramped_vel_mrad_s,
                                               s_dm_target_vel_mrad_s,
                                               elapsed_ms);

    // Keep sending MIT command periodically while enabled.
    if (s_dm_enabled && s_dm_tx_period_ms >= 10U) {
        s_dm_tx_period_ms = 0;
        vel_mrad = s_dm_ramped_vel_mrad_s;
        fb_ok = DM4310_GetFeedback(&fb);

        if (vel_mrad != 0) {
            s_dm_hold_active = 0;
            s_dm_hold_trim_nm = 0.0f;
            s_dm_hold_tff_nm = 0.0f;
            vel_rad_s = ((float)vel_mrad) / 1000.0f;
            if (vel_mrad > 0) {
                t_ff_mnm = DM4310_TFF_BOOST_MNM;
            } else {
                t_ff_mnm = (int16_t)(-DM4310_TFF_BOOST_MNM);
            }
            t_ff_nm = ((float)t_ff_mnm) / 1000.0f;
            DM4310_SetMitCommand(0.0f, vel_rad_s, 0.0f, DM4310_KD_CMD, t_ff_nm);
        } else {
            // No key pressed: keep enabled and hold position against gravity.
            if (fb_ok) {
                fb_pos_unwrapped = unwrap_pos_f32(fb.pos_rad);

                if (!s_dm_hold_active) {
                    s_dm_hold_pos_rad = fb_pos_unwrapped;
                    s_dm_hold_trim_nm = 0.0f;
                    s_dm_hold_tff_nm = 0.0f;
                    s_dm_hold_active = 1;
                }

                pos_err_rad = s_dm_hold_pos_rad - fb_pos_unwrapped;
                if (abs_f32(pos_err_rad) > DM4310_HOLD_RESET_ERR_RAD) {
                    // If deviation becomes too large, recapture target to avoid violent pullback.
                    s_dm_hold_pos_rad = fb_pos_unwrapped;
                    pos_err_rad = 0.0f;
                    s_dm_hold_trim_nm = 0.0f;
                    s_dm_hold_tff_nm = 0.0f;
                }

                if (abs_f32(pos_err_rad) < DM4310_HOLD_CAPTURE_DEADBAND_RAD) {
                    pos_err_rad = 0.0f;
                }

                trim_target_nm = s_dm_hold_trim_nm;
                if (pos_err_rad != 0.0f) {
                    trim_target_nm += pos_err_rad * DM4310_HOLD_TRIM_KI_NM_PER_RAD_S * 0.01f;
                }
                trim_target_nm = clamp_f32(trim_target_nm,
                                           -DM4310_HOLD_TRIM_MAX_NM,
                                           DM4310_HOLD_TRIM_MAX_NM);
                s_dm_hold_trim_nm = trim_target_nm;

                grav_tff_nm = DM4310_GRAV_FF_AMP_NM * cos_approx_f32(fb_pos_unwrapped + DM4310_GRAV_FF_OFFSET_RAD);
                hold_tff_target_nm = clamp_f32(grav_tff_nm + s_dm_hold_trim_nm,
                                               -DM4310_HOLD_TFF_MAX_NM,
                                               DM4310_HOLD_TFF_MAX_NM);

                s_dm_hold_tff_nm = slew_f32(s_dm_hold_tff_nm,
                                            hold_tff_target_nm,
                                            DM4310_HOLD_TFF_STEP_NM);
                hold_cmd_pos_rad = wrap_pos_to_driver_range(s_dm_hold_pos_rad);

                DM4310_SetMitCommand(hold_cmd_pos_rad,
                                     0.0f,
                                     DM4310_HOLD_KP_CMD,
                                     DM4310_HOLD_KD_CMD,
                                     s_dm_hold_tff_nm);
            } else {
                DM4310_SetMitCommand(0.0f, 0.0f, 0.0f, DM4310_HOLD_KD_CMD, 0.0f);
                s_dm_hold_trim_nm = 0.0f;
                s_dm_hold_tff_nm = 0.0f;
                s_dm_pos_unwrap_init = 0;
            }
        }
        DM4310_SendMitNow();
    }

    if (s_dm_ramped_vel_mrad_s != 0) {
        LED0 = 0;
    } else if (!s_dm_enabled) {
        LED0 = 1;
    }
#else
    (void)elapsed_ms;
#endif
}

/**
 * @brief 在串口中断中按字节解析协议
 * 格式: [HEAD(0xAA)] [LEN] [CMD] [DATA...] [CHECKSUM]
 * CHECKSUM = LEN + CMD + DATA(逐字节) 的低 8 位
 */
void Protocol_ParseByte(uint8_t byte)
{
    switch(rx_state) {
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
                rx_state = STATE_WAIT_HEADER; // 长度错误，重置
            }
            break;

        case STATE_WAIT_CMD:
            current_frame.cmd = byte;
            calc_checksum += byte;
            rx_data_index = 0;
            if (current_frame.length > 0) {
                rx_state = STATE_WAIT_DATA;
            } else {
                rx_state = STATE_WAIT_CHECKSUM;
            }
            break;

        case STATE_WAIT_DATA:
            current_frame.payload[rx_data_index++] = byte;
            calc_checksum += byte;
            if (rx_data_index >= current_frame.length) {
                rx_state = STATE_WAIT_CHECKSUM;
            }
            break;

        case STATE_WAIT_CHECKSUM:
            current_frame.checksum = byte;
            if (current_frame.checksum == calc_checksum) {
                // 校验成功，处理帧
                Protocol_HandleFrame(&current_frame);
                LED0 = !LED0; // 收到好数据翻转 LED0
            }
            rx_state = STATE_WAIT_HEADER; // 处理完毕，重新等待帧头
            break;
            
        default:
            rx_state = STATE_WAIT_HEADER;
            break;
    }
}

/**
 * @brief 处理已完整接收并校验的协议帧
 */
void Protocol_HandleFrame(ProtocolFrame_t* frame)
{
    switch (frame->cmd) {
        case CMD_SET_MOTOR_SPEED:
            // payload: [motor_id] [speed_int] [direction]
            // 为了简单，上位机将 speed (0-100 float) 乘以 1 转换成 uint8_t (0-100)
            if (frame->length >= 3) {
                uint8_t motor_id = frame->payload[0];
                float speed = (float)frame->payload[1];
                uint8_t dir = frame->payload[2];
#if defined(MOTOR86_APP)
                int16_t rpm = 0;
                (void)motor_id;
                if (speed > 100.0f) speed = 100.0f;
                if (speed < 0.0f) speed = 0.0f;

                // 0~100% mapped to 0~1200rpm for 86 stepper profile.
                rpm = (int16_t)((speed * 1200.0f) / 100.0f);
                if (dir == 1) {
                    Motor86_SetRpm(rpm);
                } else if (dir == 0) {
                    Motor86_SetRpm((int16_t)(-rpm));
                } else {
                    Motor86_SetRpm(0);
                }
#elif !defined(ENCODER_MOTOR_APP)
                if (motor_id >= MOTOR_1 && motor_id <= MOTOR_4) {
                    Motor_SetSpeed(motor_id, speed, dir);
                } else if (motor_id == 0xFF) { // 0xFF 表示控制所有电机
                    Motor_SetSpeed(MOTOR_1, speed, dir);
                    Motor_SetSpeed(MOTOR_2, speed, dir);
                    Motor_SetSpeed(MOTOR_3, speed, dir);
                    Motor_SetSpeed(MOTOR_4, speed, dir);
                }
#else
                int32_t vel_mrad = 0;
                int16_t vel_mrad_i16;
                (void)motor_id;

                if (speed > 100.0f) speed = 100.0f;
                if (speed < 0.0f) speed = 0.0f;

                if (dir == 1) {
                    vel_mrad = (int32_t)((speed * (float)DM4310_VEL_MAX_MRAD_S) / 100.0f);
                } else if (dir == 0) {
                    vel_mrad = -(int32_t)((speed * (float)DM4310_VEL_MAX_MRAD_S) / 100.0f);
                } else {
                    vel_mrad = 0;
                }

                vel_mrad_i16 = clamp_i16(vel_mrad,
                                         (int16_t)(-DM4310_VEL_MAX_MRAD_S),
                                         (int16_t)(DM4310_VEL_MAX_MRAD_S));
                s_dm_target_vel_mrad_s = vel_mrad_i16;
                s_dm_cmd_age_ms = 0;
                if (vel_mrad_i16 != 0) {
                    s_dm_hold_active = 0;
                    s_dm_hold_trim_nm = 0.0f;
                    s_dm_hold_tff_nm = 0.0f;
                }
#endif

                // LED0 低电平点亮：运行时亮，停止时灭
                if (dir != DIR_STOP && speed > 0.0f) {
                    LED0 = 0;
                } else {
                    LED0 = 1;
                }
            }
            break;

        case CMD_SET_MOTOR_VEL_MRAD:
            // payload: [motor_id] [vel_mrad_l] [vel_mrad_h], signed int16 little-endian.
            if (frame->length >= 3) {
                uint8_t motor_id = frame->payload[0];
                int16_t vel_mrad_i16 = (int16_t)((((uint16_t)frame->payload[2]) << 8) |
                                                 ((uint16_t)frame->payload[1]));
#if defined(MOTOR86_APP)
                int16_t rpm = 0;
                float vel_rad_s = ((float)vel_mrad_i16) / 1000.0f;
                (void)motor_id;
                rpm = (int16_t)(vel_rad_s * 9.5493f);
                Motor86_SetRpm(rpm);
#elif !defined(ENCODER_MOTOR_APP)
                // Fallback for old app: map signed velocity to percent + direction.
                float speed_pct = 0.0f;
                uint8_t dir = DIR_STOP;
                if (vel_mrad_i16 > 0) {
                    dir = 1;
                    speed_pct = ((float)vel_mrad_i16) / 140.0f;
                } else if (vel_mrad_i16 < 0) {
                    dir = 0;
                    speed_pct = ((float)(-vel_mrad_i16)) / 140.0f;
                }
                if (speed_pct > 100.0f) speed_pct = 100.0f;
                if (motor_id >= MOTOR_1 && motor_id <= MOTOR_4) {
                    Motor_SetSpeed(motor_id, speed_pct, dir);
                }
#else
                (void)motor_id;
                vel_mrad_i16 = clamp_i16(vel_mrad_i16,
                                         (int16_t)(-DM4310_VEL_MAX_MRAD_S),
                                         (int16_t)(DM4310_VEL_MAX_MRAD_S));
                s_dm_target_vel_mrad_s = vel_mrad_i16;
                s_dm_cmd_age_ms = 0;
                if (vel_mrad_i16 != 0) {
                    s_dm_hold_active = 0;
                    s_dm_hold_trim_nm = 0.0f;
                    s_dm_hold_tff_nm = 0.0f;
                }
#endif
                if (vel_mrad_i16 != 0) {
                    LED0 = 0;
                } else {
                    LED0 = 1;
                }
            }
            break;

        case CMD_STOP_ALL:
#if defined(MOTOR86_APP)
            Motor86_SetRpm(0);
#elif !defined(ENCODER_MOTOR_APP)
            Motor_StopAll();
#else
            s_dm_target_vel_mrad_s = 0;
            s_dm_ramped_vel_mrad_s = 0;
            s_dm_cmd_age_ms = 0;
            s_dm_hold_trim_nm = 0.0f;
            s_dm_hold_tff_nm = 0.0f;
#endif
            LED0 = 1;
            break;

        case CMD_GET_STATUS:
            Protocol_SendStatus();
            break;

        case CMD_SET_ENABLE:
            // payload: [motor_id] [enable]
            if (frame->length >= 2) {
                uint8_t motor_id = frame->payload[0];
                uint8_t enable = frame->payload[1];
#if defined(MOTOR86_APP)
                (void)motor_id;
                Motor86_Enable(enable ? 1 : 0);
                if (!enable) {
                    Motor86_SetRpm(0);
                }
#elif !defined(ENCODER_MOTOR_APP)
                Motor_Enable(motor_id, enable ? 1 : 0);
#else
                (void)motor_id;
                if (enable) {
                    if (!s_dm_enabled && !s_dm_enable_seq_busy) {
                        s_dm_req_disable = 0;
                        s_dm_req_enable = 1;
                    }
                } else {
                    if (s_dm_enabled || s_dm_req_enable || s_dm_enable_seq_busy) {
                        s_dm_req_enable = 0;
                        s_dm_req_disable = 1;
                    }
                }
                s_dm_cmd_age_ms = 0;
#endif
                if (!enable) {
                    LED0 = 1;
                }
            }
            break;

        default:
            break;
    }
}

/**
 * @brief 回复状态帧给上位机
 */
void Protocol_SendStatus(void)
{
    // 向 USART1 发送状态 (举例：固定发送 "OK")
    // 实际应用可回传编码器或实际 PWM 占空比等
    uint8_t tx_buf[] = {0xAA, 0x02, 0x83, 0x4F, 0x4B, 0x1F}; // HEAD(AA) LEN(02) CMD(83) 'O' 'K' CS
    for(int i=0; i<sizeof(tx_buf); i++){
        while((USART1->SR&0X40)==0); // 等待发送结束
        USART1->DR = tx_buf[i];
    }
}
