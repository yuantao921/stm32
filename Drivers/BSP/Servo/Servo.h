/**
 ****************************************************************************************************
 * @file        Servo.h
 * @author      改写自STM32F103版本
 * @version     V1.0
 * @date        2024
 * @brief       舵机控制驱动代码 (DS3218舵机) - STM32F407 HAL库版本
 * @note
 *              实验平台:正点原子 探索者 F407开发板
 *              - PA0控制左右舵机 (TIM2_CH1)
 *              - PA1控制上下舵机 (TIM2_CH2)
 ****************************************************************************************************
 */

#ifndef __SERVO_H
#define __SERVO_H

#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/
/* 舵机控制 引脚 和 定时器 定义 */

/* 舵机控制使用 TIM2
 * PA0 -> TIM2_CH1 -> 左右舵机
 * PA1 -> TIM2_CH2 -> 上下舵机
 */
#define SERVO_GPIO_PORT                     GPIOA
#define SERVO_LEFT_GPIO_PIN                 GPIO_PIN_0      /* 左右舵机引脚 PA0 */
#define SERVO_UP_GPIO_PIN                   GPIO_PIN_1      /* 上下舵机引脚 PA1 */
#define SERVO_GPIO_CLK_ENABLE()             do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)     /* PA口时钟使能 */
#define SERVO_GPIO_AF                       GPIO_AF1_TIM2   /* TIM2复用功能 */

#define SERVO_TIMX                          TIM2
#define SERVO_LEFT_CHANNEL                  TIM_CHANNEL_1   /* 左右舵机通道 */
#define SERVO_UP_CHANNEL                    TIM_CHANNEL_2   /* 上下舵机通道 */
#define SERVO_TIMX_CLK_ENABLE()             do{ __HAL_RCC_TIM2_CLK_ENABLE(); }while(0)      /* TIM2 时钟使能 */

/* 舵机参数定义 (DS3218)
 * 周期: 20ms (50Hz)
 * 脉宽范围: 0.5ms ~ 2.5ms (对应 0度 ~ 180度)
 * 使用1MHz计数频率，则:
 *   0.5ms = 500 counts
 *   2.5ms = 2500 counts
 *   20ms  = 20000 counts
 */
#define PWM_PERIOD                         19999U           /* PWM周期 (20ms @ 1MHz = 20000 counts, 实际ARR=19999) */
#define PWM_MIN                            500U             /* 最小脉宽 (0.5ms, 0度) */
#define PWM_MID                            1500U            /* 中位脉宽 (1.5ms, 90度) */
#define PWM_MAX                            2500U            /* 最大脉宽 (2.5ms, 180度) */

/******************************************************************************************/

void Servo_TIM2_Init(void);                                      /* 初始化 TIM2 双路舵机PWM */
void Servo_SetAngle(uint8_t ch, float angle);                   /* ch=0或1，角度0~180 */
void Servo_SetCenterOffset(uint8_t ch, float offset);            /* 中位校准 */
float Servo_GetCenterOffset(uint8_t ch);                          /* 获取校准值 */
void Servo_SelfTest_Dual(void);                                  /* 双路同步自检（0°?180°?90°） */
void Servo_SetAngleRange(uint8_t ch, float min_angle, float max_angle); /* 自定义物理角度范围 */

/* 兼容旧接口 */
#define servo_init()                     Servo_TIM2_Init()
#define servo_set_angle(servo_id, angle) Servo_SetAngle(servo_id, angle)
#define servo_self_check()               Servo_SelfTest_Dual()

#endif

