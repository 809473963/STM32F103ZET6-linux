#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "stm32f10x.h"

// 协议帧头和特定命令
#define PROTOCOL_HEADER 0xAA

#define CMD_SET_MOTOR_SPEED  0x01
#define CMD_STOP_ALL         0x02
#define CMD_GET_STATUS       0x03
#define CMD_SET_ENABLE       0x04

// 内部处理状态机
typedef enum {
    STATE_WAIT_HEADER = 0,
    STATE_WAIT_LENGTH,
    STATE_WAIT_CMD,
    STATE_WAIT_DATA,
    STATE_WAIT_CHECKSUM
} ProtocolState_t;

#define MAX_PAYLOAD_LEN 32

typedef struct {
    uint8_t header;
    uint8_t length;
    uint8_t cmd;
    uint8_t payload[MAX_PAYLOAD_LEN];
    uint8_t checksum;
} ProtocolFrame_t;

void Protocol_Init(void);
void Protocol_ParseByte(uint8_t byte);
void Protocol_HandleFrame(ProtocolFrame_t* frame);

// 用于发送状态回复
void Protocol_SendStatus(void);

#endif
