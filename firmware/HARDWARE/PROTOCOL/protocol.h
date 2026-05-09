#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "stm32f10x.h"

// 协议帧头和特定命令
#define PROTOCOL_HEADER 0xAA

#define CMD_SET_MOTOR_SPEED  0x01
#define CMD_STOP_ALL         0x02
#define CMD_GET_STATUS       0x03
#define CMD_SET_ENABLE       0x04
#define CMD_SET_MOTOR_VEL_MRAD 0x05

// ARM DM extended commands.
#define CMD_GATEWAY_CONTROL  0x20
#define CMD_DM_SCAN_IDS      0x21
#define CMD_DM_SET_ID        0x22
#define CMD_DM_SAVE_ZERO     0x23
#define CMD_SET_GRAVITY_COMP 0x24
#define CMD_REZERO_GRAVITY   0x25
#define CMD_DM_REG_READ      0x26
#define CMD_DM_REG_WRITE     0x27
#define CMD_DM_REG_SAVE      0x28
#define CMD_DM_RAW_CAN       0x29

#define CMD_STATUS_REPLY     0x83
#define CMD_DM_SCAN_IDS_REPLY 0xA1
#define CMD_DM_SET_ID_REPLY   0xA2
#define CMD_DM_REG_READ_REPLY  0xA6
#define CMD_DM_REG_WRITE_REPLY 0xA7

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
void Protocol_TickMs(uint16_t elapsed_ms);

// 用于发送状态回复
void Protocol_SendStatus(void);

#endif
