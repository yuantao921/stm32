/**
 ****************************************************************************************************
 * @file        servo_track.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-xx
 * @brief       舵机光斑追踪模块 - STM32F407适配版本
 * @note        功能：根据光斑坐标自动控制舵机追踪，支持手动/自动模式切换
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 探索者 F407开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#ifndef __SERVO_TRACK_H
#define __SERVO_TRACK_H

#include "./SYSTEM/sys/sys.h"
#include "./BSP/Servo/Servo.h"
#include "./BSP/SERVO_TRACK/spot_detect.h"

//////////////////////////////////////////////////////////////////////////////////
// ALIENTEK STM32F407
// 舵机光斑追踪模块
// 功能：根据光斑坐标自动控制舵机追踪，支持手动/自动模式切换
//////////////////////////////////////////////////////////////////////////////////

// ==================== 工作模式定义 ====================
typedef enum {
    SERVO_MODE_MANUAL = 0,      // 手动模式（按键控制）
    SERVO_MODE_AUTO_TRACK       // 自动追踪模式
} servo_track_mode_t;

// ==================== 配置参数结构体 ====================
typedef struct {
    uint8_t pan_channel;        // 水平舵机通道号 (0~1, 对应TIM2_CH1/CH2)
    uint8_t tilt_channel;       // 垂直舵机通道号 (0~1, 对应TIM2_CH1/CH2)
    uint16_t img_width;         // 图像宽度 (像素)
    uint16_t img_height;        // 图像高度 (像素)
    float smooth_factor;        // 平滑系数 (0~1, 越小越平滑)
    uint16_t dead_zone;         // 中心死区半径 (像素)
    float min_angle_change;     // 最小角度变化阈值 (度)，小于此值不移动舵机
} servo_track_config_t;

// ==================== 状态信息结构体 ====================
typedef struct {
    float pan_angle;            // 当前水平角度
    float tilt_angle;           // 当前垂直角度
    uint16_t target_x;          // 目标X坐标
    uint16_t target_y;          // 目标Y坐标
    uint8_t tracking;           // 是否正在追踪
    uint32_t lost_frames;       // 丢失帧数计数
    uint8_t returning_to_center;// 是否正在回中，1=回中中，0=不回中
} servo_track_state_t;

// ==================== 函数声明 ====================

/**
 * @brief  初始化舵机追踪模块
 * @param  pan_channel: 水平舵机通道号 (0~1)
 * @param  tilt_channel: 垂直舵机通道号 (0~1)
 * @param  img_width: 图像宽度 (如 320)
 * @param  img_height: 图像高度 (如 240)
 * @retval None
 */
void ServoTrack_Init(uint8_t pan_channel, uint8_t tilt_channel, uint16_t img_width, uint16_t img_height);

/**
 * @brief  设置工作模式
 * @param  mode: SERVO_MODE_MANUAL 或 SERVO_MODE_AUTO_TRACK
 * @retval None
 */
void ServoTrack_SetMode(servo_track_mode_t mode);

/**
 * @brief  获取当前工作模式
 * @retval 当前模式
 */
servo_track_mode_t ServoTrack_GetMode(void);

/**
 * @brief  追踪处理函数（主循环调用）
 * @note   自动模式下根据光斑位置控制舵机，手动模式下不执行
 * @retval None
 */
void ServoTrack_Process(void);

/**
 * @brief  手动控制舵机角度（增量控制）
 * @param  pan_delta: 水平角度增量 (度)，正值右转，负值左转
 * @param  tilt_delta: 垂直角度增量 (度)，正值下转，负值上转
 * @retval None
 */
void ServoTrack_ManualControl(float pan_delta, float tilt_delta);

/**
 * @brief  手动设置舵机角度（绝对控制）
 * @param  pan_angle: 水平角度 (0~180度)
 * @param  tilt_angle: 垂直角度 (0~180度)
 * @retval None
 */
void ServoTrack_SetAngles(float pan_angle, float tilt_angle);

/**
 * @brief  设置平滑系数
 * @param  factor: 平滑系数 (0~1)，0表示最平滑，1表示立即响应
 * @note   建议值：0.2~0.5
 * @retval None
 */
void ServoTrack_SetSmoothFactor(float factor);

/**
 * @brief  设置中心死区半径
 * @param  zone: 死区半径 (像素)，光斑在中心死区内时不移动舵机
 * @note   建议值：5~15
 * @retval None
 */
void ServoTrack_SetDeadZone(uint16_t zone);

/**
 * @brief  设置最小角度变化阈值
 * @param  threshold: 角度变化阈值 (度)，小于此值不移动舵机
 * @note   建议值：0.5~2.0，用于避免微小抖动
 * @retval None
 */
void ServoTrack_SetMinAngleChange(float threshold);

/**
 * @brief  获取当前水平角度
 * @retval 当前水平角度 (0~180度)
 */
float ServoTrack_GetPanAngle(void);

/**
 * @brief  获取当前垂直角度
 * @retval 当前垂直角度 (0~180度)
 */
float ServoTrack_GetTiltAngle(void);

/**
 * @brief  获取追踪状态信息
 * @retval 状态信息结构体指针
 */
servo_track_state_t* ServoTrack_GetState(void);

/**
 * @brief  重置舵机到中心
 * @retval None
 */
void ServoTrack_Reset(void);

/**
 * @brief  打印调试信息
 * @retval None
 */
void ServoTrack_PrintDebugInfo(void);

/**
 * @brief  设置舵机轴方向反转（适配机械安装方向差异）
 * @param  pan_invert: 1=水平轴反转(角度变为180-pan)，0=不反转
 * @param  tilt_invert: 1=垂直轴反转(角度变为180-tilt)，0=不反转
 */
void ServoTrack_SetAxisInvert(uint8_t pan_invert, uint8_t tilt_invert);

#endif

