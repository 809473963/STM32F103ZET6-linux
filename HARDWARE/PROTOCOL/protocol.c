#include "protocol.h"
#include "motor.h"
#include "usart.h" // 使用现有的 USART 发送函数
#include "led.h"   // 闪烁 LED 指示收发状态

static ProtocolState_t rx_state = STATE_WAIT_HEADER;
static ProtocolFrame_t current_frame;
static uint8_t rx_data_index = 0;
static uint8_t calc_checksum = 0;

void Protocol_Init(void)
{
    rx_state = STATE_WAIT_HEADER;
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
                if (motor_id >= MOTOR_1 && motor_id <= MOTOR_4) {
                    Motor_SetSpeed(motor_id, speed, dir);
                } else if (motor_id == 0xFF) { // 0xFF 表示控制所有电机
                    Motor_SetSpeed(MOTOR_1, speed, dir);
                    Motor_SetSpeed(MOTOR_2, speed, dir);
                    Motor_SetSpeed(MOTOR_3, speed, dir);
                    Motor_SetSpeed(MOTOR_4, speed, dir);
                }

                // LED0 低电平点亮：运行时亮，停止时灭
                if (dir != DIR_STOP && speed > 0.0f) {
                    LED0 = 0;
                } else {
                    LED0 = 1;
                }
            }
            break;

        case CMD_STOP_ALL:
            Motor_StopAll();
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
                Motor_Enable(motor_id, enable ? 1 : 0);
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
