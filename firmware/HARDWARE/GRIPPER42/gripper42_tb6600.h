#ifndef __GRIPPER42_TB6600_H
#define __GRIPPER42_TB6600_H

#include "stm32f10x.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GRIPPER42_STATE_UNHOMED = 0,
    GRIPPER42_STATE_READY = 1,
    GRIPPER42_STATE_FAULT = 2
} Gripper42State_t;

typedef struct {
    uint8_t state;
    uint8_t zero_valid;
    int32_t pos_steps;
    int32_t target_steps;
    int32_t min_steps;
    int32_t max_steps;
    uint16_t margin_steps;
    uint16_t run_speed_steps_s;
} Gripper42Status_t;

void Gripper42_Init(void);
void Gripper42_TickMs(uint16_t elapsed_ms);

void Gripper42_Jog(uint8_t dir, uint8_t speed_pct);
void Gripper42_Stop(void);
void Gripper42_SetOpenPercent(uint8_t open_pct);

uint8_t Gripper42_SetZero(void);
uint8_t Gripper42_SaveConfig(void);
uint8_t Gripper42_TrustLastZero(void);

void Gripper42_GetStatus(Gripper42Status_t *out);

#ifdef __cplusplus
}
#endif

#endif
