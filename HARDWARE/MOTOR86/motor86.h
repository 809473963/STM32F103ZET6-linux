#ifndef __MOTOR86_H
#define __MOTOR86_H

#include "stm32f10x.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 86 stepper driver wiring:
 *  PUL+ -> PA2 (TIM2_CH3 PWM)
 *  DIR+ -> PA3 (GPIO)
 *  ENA+ -> PA4 (GPIO, active low enable)
 */

#define MOTOR86_PPR_DEFAULT 1600U
#define MOTOR86_EN_ACTIVE_LEVEL 0U

void Motor86_Init(void);
void Motor86_Enable(uint8_t enable);
void Motor86_SetPpr(uint16_t ppr);
uint16_t Motor86_GetPpr(void);
void Motor86_SetRpm(int16_t rpm);
int16_t Motor86_GetRpm(void);

#ifdef __cplusplus
}
#endif

#endif
