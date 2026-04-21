#ifndef __DM4310_CAN_H
#define __DM4310_CAN_H

#include "stm32f10x.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t valid;
    uint8_t motor_id;
    uint8_t err;
    float pos_rad;
    float vel_rad_s;
    float tau_nm;
    uint8_t mos_temp;
    uint8_t rotor_temp;
    uint32_t rx_count;
} DM4310_Feedback_t;

uint8_t DM4310_Init(uint16_t motor_id);
void DM4310_SetMotorId(uint16_t motor_id);
uint16_t DM4310_GetMotorId(void);

void DM4310_SetMitCommand(float p_des, float v_des, float kp, float kd, float t_ff);
uint8_t DM4310_SendMitNow(void);

void DM4310_EnterMitMode(void);
void DM4310_EnableCompat(void);
void DM4310_Disable(void);
void DM4310_ClearError(void);
void DM4310_SaveZero(void);

uint8_t DM4310_HasFeedback(void);
uint8_t DM4310_GetFeedback(DM4310_Feedback_t *out);

#ifdef __cplusplus
}
#endif

#endif