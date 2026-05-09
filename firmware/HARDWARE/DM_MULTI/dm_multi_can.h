#ifndef __DM_MULTI_CAN_H
#define __DM_MULTI_CAN_H

#include "stm32f10x.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DM_MULTI_MAX_MOTORS 8U

#define DM_MULTI_RID_FEEDBACK_ID 0x07U
#define DM_MULTI_RID_RECV_ID 0x08U
#define DM_MULTI_RID_CTRL_MODE 0x0AU

#define DM_SPEED_MODE_ID_OFFSET 0x200U

#define DM_CTRL_MODE_MIT 1U
#define DM_CTRL_MODE_VEL 3U

typedef enum {
    DM_MULTI_TYPE_DM4310 = 0,
    DM_MULTI_TYPE_DM3510 = 1,
    DM_MULTI_TYPE_M24 = 2
} DM_MultiMotorType_t;

typedef struct {
    uint16_t can_id;
    DM_MultiMotorType_t type;
    float q_max;
    float dq_max;
    float tau_max;
} DM_MultiMotorConfig_t;

typedef struct {
    uint8_t valid;
    uint8_t online;
    uint16_t can_id;
    uint8_t motor_id;
    uint8_t err;
    float pos_rad;
    float vel_rad_s;
    float tau_nm;
    uint8_t mos_temp;
    uint8_t rotor_temp;
    uint32_t rx_count;
} DM_MultiFeedback_t;

typedef struct {
    uint8_t valid;
    uint8_t op;
    uint16_t can_id;
    uint8_t rid;
    uint32_t value_u32;
} DM_MultiRegAck_t;

uint8_t DM_Multi_Init(const DM_MultiMotorConfig_t *cfg, uint8_t cfg_count);
uint8_t DM_Multi_IsReady(void);

uint8_t DM_Multi_Enable(uint16_t can_id);
uint8_t DM_Multi_Disable(uint16_t can_id);
uint8_t DM_Multi_ClearError(uint16_t can_id);
uint8_t DM_Multi_SaveZero(uint16_t can_id);

uint8_t DM_Multi_EnableCompat(uint16_t can_id);
uint8_t DM_Multi_SetControlModeMit(uint16_t can_id);

uint8_t DM_Multi_SetMit(uint16_t can_id, float p_des, float v_des, float kp, float kd, float t_ff);
uint8_t DM_Multi_SetVel(uint16_t can_id, float vel);
uint8_t DM_Multi_SetControlModeVel(uint16_t can_id);

uint8_t DM_Multi_RequestStatus(uint16_t can_id);
uint8_t DM_Multi_ReadRegister(uint16_t can_id, uint8_t rid);
uint8_t DM_Multi_WriteRegisterU32(uint16_t can_id, uint8_t rid, uint32_t value);
uint8_t DM_Multi_SendRawCAN(uint16_t std_id, const uint8_t data[8]);
uint8_t DM_Multi_SetMotorId(uint16_t current_id, uint16_t new_id, uint8_t update_feedback_id);

uint8_t DM_Multi_GetFeedback(uint16_t can_id, DM_MultiFeedback_t *out);
uint8_t DM_Multi_GetFeedbackByIndex(uint8_t index, DM_MultiFeedback_t *out);

void DM_Multi_ClearSeenIds(void);
uint8_t DM_Multi_GetSeenIds(uint8_t *out_ids, uint8_t max_count);
uint32_t DM_Multi_GetSeenIdMask(void);

void DM_Multi_ClearLastRegAck(void);
uint8_t DM_Multi_GetLastRegAck(DM_MultiRegAck_t *out);

#ifdef __cplusplus
}
#endif

#endif
