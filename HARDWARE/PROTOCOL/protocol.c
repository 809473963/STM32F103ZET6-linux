#include "protocol.h"
#if defined(MOTOR86_APP)
#include "motor86.h"
#elif !defined(ENCODER_MOTOR_APP)
#include "motor.h"
#else
#include "encoder_motor.h"
#endif
#include "usart.h" // 使用现有的 USART 发送函数
#include "led.h"

static ProtocolState_t rx_state = STATE_WAIT_HEADER;
static ProtocolFrame_t current_frame;
static uint8_t rx_data_index = 0;
static uint8_t calc_checksum = 0;

#ifndef DIR_STOP
#define DIR_STOP 2
#endif

#ifdef ENCODER_MOTOR_APP
#define ENCODER_VEL_MAX_MRAD_S 14000
#define ENCODER_DUTY_MAX_PERCENT 100

static int16_t clamp_i16(int32_t x, int16_t lo, int16_t hi)
{
    if (x < (int32_t)lo) return lo;
    if (x > (int32_t)hi) return hi;
    return (int16_t)x;
}

static void encoder_apply_speed_by_protocol_id(uint8_t motor_id, int16_t duty_percent)
{
    duty_percent = clamp_i16(duty_percent,
                             (int16_t)(-ENCODER_DUTY_MAX_PERCENT),
                             (int16_t)(ENCODER_DUTY_MAX_PERCENT));

    if (motor_id == ENCODER_MOTOR_ID_ALL) {
        EncoderMotor_SetOutputPercentById(ENCODER_MOTOR_ID_ALL, duty_percent);
    } else {
        EncoderMotor_SetOutputPercentById(motor_id, duty_percent);
    }
}
#endif

void Protocol_Init(void)
{
    rx_state = STATE_WAIT_HEADER;
#ifdef ENCODER_MOTOR_APP
    EncoderMotor_InitAll();
#endif
}

void Protocol_TickMs(uint16_t elapsed_ms)
{
    (void)elapsed_ms;
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
                int16_t duty = 0;
                if (speed > 100.0f) speed = 100.0f;
                if (speed < 0.0f) speed = 0.0f;
                if (dir == 1) {
                    duty = (int16_t)speed;
                } else if (dir == 0) {
                    duty = (int16_t)(-speed);
                } else {
                    duty = 0;
                }
                encoder_apply_speed_by_protocol_id(motor_id, duty);
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
                int16_t signed_percent;
                vel_mrad_i16 = clamp_i16(vel_mrad_i16,
                                         (int16_t)(-ENCODER_VEL_MAX_MRAD_S),
                                         (int16_t)(ENCODER_VEL_MAX_MRAD_S));
                signed_percent = (int16_t)(((int32_t)vel_mrad_i16 * ENCODER_DUTY_MAX_PERCENT) /
                                           ENCODER_VEL_MAX_MRAD_S);
                encoder_apply_speed_by_protocol_id(motor_id, signed_percent);
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
            encoder_apply_speed_by_protocol_id(ENCODER_MOTOR_ID_ALL, 0);
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
                if (motor_id == ENCODER_MOTOR_ID_ALL) {
                    EncoderMotor_EnableById(ENCODER_MOTOR_ID_ALL, enable ? 1U : 0U);
                } else {
                    EncoderMotor_EnableById(motor_id, enable ? 1U : 0U);
                }
                if (!enable) {
                    encoder_apply_speed_by_protocol_id(motor_id, 0);
                }
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
    // Legacy fixed status payload kept for compatibility with host scripts.
    uint8_t tx_buf[] = {0xAA, 0x02, 0x83, 0x4F, 0x4B, 0x1F}; // HEAD(AA) LEN(02) CMD(83) 'O' 'K' CS
    for(int i=0; i<sizeof(tx_buf); i++){
        while((USART1->SR&0X40)==0); // 等待发送结束
        USART1->DR = tx_buf[i];
    }
}
