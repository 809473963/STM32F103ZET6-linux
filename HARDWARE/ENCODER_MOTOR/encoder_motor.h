#ifndef __ENCODER_MOTOR_H
#define __ENCODER_MOTOR_H

#include "stm32f10x.h"
#include <stdint.h>

// BTS7960 (IBT-2) route:
// L_PWM -> PB8 (TIM4_CH3), R_PWM -> PB9 (TIM4_CH4)

// Motor IDs used by protocol layer.
#define ENCODER_MOTOR_ID_1   1U
#define ENCODER_MOTOR_ID_2   2U
#define ENCODER_MOTOR_ID_3   3U
#define ENCODER_MOTOR_ID_ALL 0xFFU

void EncoderMotor_InitAll(void);
void EncoderMotor_EnableById(uint8_t motor_id, uint8_t en);
void EncoderMotor_SetSpeedById(uint8_t motor_id, int16_t speed);
void EncoderMotor_SetOutputPercentById(uint8_t motor_id, int16_t percent);
int16_t EncoderMotor_GetOutputPercentById(uint8_t motor_id);
uint8_t EncoderMotor_IsEnabledById(uint8_t motor_id);
uint8_t EncoderMotor_GetDirectionForwardById(uint8_t motor_id);
uint16_t EncoderMotor_GetPulseHzById(uint8_t motor_id);
int32_t EncoderMotor_GetEncoderCountById(uint8_t motor_id);
void EncoderMotor_ResetEncoderById(uint8_t motor_id);

// BTS7960 low-level debug helpers.
void Set_Motor_Speed(int speed);
void BTS7960_SoftStart_Test(void);

// Current sync requirement: motor1 and motor2 run together.
void EncoderMotor_EnableSync12(uint8_t en);
void EncoderMotor_SetSync12OutputPercent(int16_t percent);

// Legacy single-motor API kept for backward compatibility (maps to motor2 BTS route).
void EncoderMotor_Init(void);
void EncoderMotor_Enable(uint8_t en);
void EncoderMotor_SetOutputPercent(int16_t percent);
int16_t EncoderMotor_GetOutputPercent(void);
uint8_t EncoderMotor_IsEnabled(void);
uint8_t EncoderMotor_GetDirectionForward(void);
uint16_t EncoderMotor_GetPulseHz(void);
int32_t EncoderMotor_GetEncoderCount(void);
void EncoderMotor_ResetEncoder(void);

#endif
