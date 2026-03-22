#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

// 电机 ID 定义
#define MOTOR_1 1
#define MOTOR_2 2
#define MOTOR_3 3
#define MOTOR_4 4

// 电机方向定义
#define DIR_FORWARD  1
#define DIR_BACKWARD 0
#define DIR_STOP     2

// EN 极性：1=高电平使能，0=低电平使能
// 常见 42 步进驱动器（光耦输入）通常更接近低电平使能。
#define MOTOR1_EN_ACTIVE_LEVEL 0

// 速度映射到步进脉冲周期（us）
// speed 越大，pulse_us 越小，转速越快。
#define MOTOR1_PULSE_US_MAX 2000U
#define MOTOR1_PULSE_US_MIN 120U

/*
 * 当前接线（单电机步进驱动）:
 * PA8 -> PUL+
 * PA7 -> DIR+
 * PA2 -> ENA+
 *
 * 说明:
 * - MOTOR_1 使用 Step/Dir/Enable 控制
 * - MOTOR_2/3/4 预留，当前未实现
 */

void Motor_Init(void);
void Motor_Enable(uint8_t motor_id, uint8_t enable);
void Motor_SetSpeed(uint8_t motor_id, float speed, uint8_t direction);
void Motor_Service(void);
void Motor_StopAll(void);

#endif
