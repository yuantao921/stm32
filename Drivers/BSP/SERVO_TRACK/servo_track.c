/**
 ****************************************************************************************************
 * @file        servo_track.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-xx
 * @brief       舵机光斑追踪模块实现 - STM32F407适配版本
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

#include "./BSP/SERVO_TRACK/servo_track.h"
#include "./SYSTEM/usart/usart.h"
#include <math.h>

//////////////////////////////////////////////////////////////////////////////////
// ALIENTEK STM32F407
// 舵机光斑追踪模块 - 实现
//////////////////////////////////////////////////////////////////////////////////

// ==================== 方向测试开关与参数 ====================
#ifndef SERVO_DIR_TEST
#define SERVO_DIR_TEST 1      // 方向/指令调试输出开关(1=开启, 0=关闭)
#endif

#ifndef DIR_MARGIN_PX
#define DIR_MARGIN_PX 20      // 中心判定的像素边界(左右/上下±DIR_MARGIN_PX 内视为CENTER)
#endif

// ==================== 内部变量 ====================
static servo_track_config_t config;         // 配置参数
static servo_track_state_t state;           // 状态信息
static servo_track_mode_t current_mode;     // 当前工作模式

// 调试输出控制
static uint32_t debug_counter = 0;
static uint8_t debug_enable = 1;

// 轴反向开关（适配机械安装方向差异），0=正常，1=反向（角度变为 180-角度）
static uint8_t pan_invert_flag = 0;  // PAN(水平)轴
static uint8_t tilt_invert_flag = 0; // TILT(垂直)轴

// ==================== 内部函数声明 ====================
static void coord_to_angles(uint16_t x, uint16_t y, float *pan, float *tilt);
static void limit_angle(float *angle);
static uint8_t in_dead_zone(uint16_t x, uint16_t y);

#if SERVO_DIR_TEST
// 方向分类: X轴: LEFT/CENTER/RIGHT; Y轴: TOP/CENTER/BOTTOM
typedef enum { X_CENTER = 0, X_LEFT = 1, X_RIGHT = 2 } x_dir_t;
typedef enum { Y_CENTER = 0, Y_TOP = 1, Y_BOTTOM = 2 } y_dir_t;

static const char* x_dir_name(x_dir_t d) {
    switch(d) {
        case X_LEFT:  return "LEFT";
        case X_RIGHT: return "RIGHT";
        default:      return "CENTER";
    }
}

static const char* y_dir_name(y_dir_t d) {
    switch(d) {
        case Y_TOP:    return "TOP";
        case Y_BOTTOM: return "BOTTOM";
        default:       return "CENTER";
    }
}

static x_dir_t last_x_dir = (x_dir_t)255; // 未初始化标记
static y_dir_t last_y_dir = (y_dir_t)255;
#endif

// ==================== 初始化函数 ====================
void ServoTrack_Init(uint8_t pan_channel, uint8_t tilt_channel, uint16_t img_width, uint16_t img_height)
{
    // 配置参数
    config.pan_channel = pan_channel;
    config.tilt_channel = tilt_channel;
    config.img_width = img_width;
    config.img_height = img_height;
    config.smooth_factor = 0.3f;        // 默认平滑系数，控制每帧移动的比例，值越大：响应越快，但可能抖动
    config.dead_zone = 10;              // 默认死区半径10像素
    config.min_angle_change = 1.0f;     // 默认最小角度变化1度

    // 初始状态
    state.pan_angle = 90.0f;            // 初始中心
    state.tilt_angle = 90.0f;           // 初始中心
    state.target_x = img_width / 2;
    state.target_y = img_height / 2;
    state.tracking = 0;
    state.lost_frames = 0;
    state.returning_to_center = 0;      // 初始不回中

    // 初始模式
    current_mode = SERVO_MODE_MANUAL;

    // 设置舵机到中心
    Servo_SetAngle(config.pan_channel, state.pan_angle);
    Servo_SetAngle(config.tilt_channel, state.tilt_angle);

    printf("[TRACK] Init OK - Pan:CH%d Tilt:CH%d Image:%dx%d\r\n",
           pan_channel, tilt_channel, img_width, img_height);
#if SERVO_DIR_TEST
    last_x_dir = (x_dir_t)255;
    last_y_dir = (y_dir_t)255;
#endif
}

// ==================== 模式控制 ====================
void ServoTrack_SetMode(servo_track_mode_t mode)
{
    if(current_mode != mode)
    {
        current_mode = mode;
        printf("[TRACK] Mode changed to: %s\r\n",
               mode == SERVO_MODE_MANUAL ? "MANUAL" : "AUTO_TRACK");

        // 切换到自动模式时重置丢失计数和回中状态
        if(mode == SERVO_MODE_AUTO_TRACK)
        {
            state.lost_frames = 0;
            state.returning_to_center = 0;
        }
    }
}

servo_track_mode_t ServoTrack_GetMode(void)
{
    return current_mode;
}

// ==================== 追踪处理函数 ====================
void ServoTrack_Process(void)
{
    static uint32_t move_count = 0;  
    static uint8_t lost_log_count = 0;

    // 只在自动模式下执行
    if(current_mode != SERVO_MODE_AUTO_TRACK)
    {
        return;
    }

    // 检查是否检测到光斑
    if(SPOT_Detect_IsFound())
    {
        uint16_t spot_x, spot_y;
        float target_pan, target_tilt;
        float center_x, center_y;

        // 获取光斑坐标
        SPOT_Detect_GetCenter(&spot_x, &spot_y);

        // 中心坐标
        center_x = config.img_width / 2.0f;   // 160
        center_y = config.img_height / 2.0f;  // 120

        // 更新目标坐标
        state.target_x = spot_x;
        state.target_y = spot_y;

        // 坐标转换为角度（先计算，用于调试）
        coord_to_angles(spot_x, spot_y, &target_pan, &target_tilt);

        // 调试：前30帧都输出坐标和目标角度
        if(move_count < 30)
        {
            printf("[TRACK] Detect: xy:(%d,%d) -> Target:(%.1f,%.1f) Current:(%.1f,%.1f)\r\n",
                   spot_x, spot_y, target_pan, target_tilt,
                   state.pan_angle, state.tilt_angle);
        }

        // 检查是否在死区内
        if(in_dead_zone(spot_x, spot_y))
        {
            // 在死区内，不移动舵机
            if(debug_enable && (debug_counter % 30 == 0))
            {
                printf("[TRACK] In dead zone, holding position\r\n");
            }
        }
        else
        {
            float new_pan, new_tilt;
            float pan_delta, tilt_delta;
#if SERVO_DIR_TEST
            x_dir_t xdir;
            y_dir_t ydir;
            const char* pan_cmd;
            const char* tilt_cmd;
#endif

            // 平滑滤波（一阶低通滤波）
            new_pan = state.pan_angle * (1.0f - config.smooth_factor) +
                      target_pan * config.smooth_factor;
            new_tilt = state.tilt_angle * (1.0f - config.smooth_factor) +
                       target_tilt * config.smooth_factor;

            // 限幅保护
            limit_angle(&new_pan);
            limit_angle(&new_tilt);

            // 计算角度变化量
            pan_delta = fabsf(new_pan - state.pan_angle);
            tilt_delta = fabsf(new_tilt - state.tilt_angle);

#if SERVO_DIR_TEST
            // 方向分类与指令方向打印
            xdir = X_CENTER;
            ydir = Y_CENTER;
            if(spot_x + DIR_MARGIN_PX < (uint16_t)center_x) xdir = X_LEFT;
            else if(spot_x > (uint16_t)(center_x + DIR_MARGIN_PX)) xdir = X_RIGHT;
            else xdir = X_CENTER;

            if(spot_y + DIR_MARGIN_PX < (uint16_t)center_y) ydir = Y_TOP;
            else if(spot_y > (uint16_t)(center_y + DIR_MARGIN_PX)) ydir = Y_BOTTOM;
            else ydir = Y_CENTER;

            pan_cmd = (new_pan > state.pan_angle + 1e-3f) ? "RIGHT" :
                      (new_pan + 1e-3f < state.pan_angle ? "LEFT" : "HOLD");
            tilt_cmd = (new_tilt > state.tilt_angle + 1e-3f) ? "DOWN" :
                       (new_tilt + 1e-3f < state.tilt_angle ? "UP"   : "HOLD");

            printf("[DIR] xy:(%u,%u) pos:%s|%s cmd:PAN=%s TILT=%s\r\n",
                   spot_x, spot_y, x_dir_name(xdir), y_dir_name(ydir), pan_cmd, tilt_cmd);

            // 当某轴从非CENTER进入CENTER时，打印进入中心的方向信息
            if(last_x_dir != (x_dir_t)255 && xdir == X_CENTER && last_x_dir != X_CENTER) {
                printf("[DIR] X CROSS: %s -> CENTER\r\n", x_dir_name(last_x_dir));
            }
            if(last_y_dir != (y_dir_t)255 && ydir == Y_CENTER && last_y_dir != Y_CENTER) {
                printf("[DIR] Y CROSS: %s -> CENTER\r\n", y_dir_name(last_y_dir));
            }
            last_x_dir = xdir;
            last_y_dir = ydir;
#endif

            // 仅当角度变化超过阈值时才移动舵机
            if(pan_delta >= config.min_angle_change || tilt_delta >= config.min_angle_change)
            {
                state.pan_angle = new_pan;
                state.tilt_angle = new_tilt;

                // 控制舵机
                Servo_SetAngle(config.pan_channel, state.pan_angle);
                Servo_SetAngle(config.tilt_channel, state.tilt_angle);

                // 详细调试输出（前30次移动都输出）
                if(move_count < 30)
                {
                    printf("[TRACK] #%d xy:(%d,%d) Target:(%.1f,%.1f) => CH%d=%.1f CH%d=%.1f Delta:(%.1f,%.1f)\r\n",
                           move_count, spot_x, spot_y, target_pan, target_tilt,
                           config.pan_channel, state.pan_angle,
                           config.tilt_channel, state.tilt_angle,
                           pan_delta, tilt_delta);
                    move_count++;
                }
                else if(move_count % 20 == 0)  //之后每20次输出一次
                {
                    printf("[TRACK] #%d xy:(%d,%d) => CH%d=%.1f CH%d=%.1f\r\n",
                           move_count, spot_x, spot_y,
                           config.pan_channel, state.pan_angle,
                           config.tilt_channel, state.tilt_angle);
                    move_count++;
                }
                else
                {
                    move_count++;
                }
            }
        }

        // 标记正在追踪
        state.tracking = 1;
        state.lost_frames = 0;
        lost_log_count = 0;  //重置lost计数

        // 如果之前在回中，立即停止回中并恢复追踪
        if(state.returning_to_center)
        {
            state.returning_to_center = 0;
            printf("[TRACK] Target reacquired! Canceling return to center, resuming tracking\r\n");
        }
    }
    else
    {
        // 光斑丢失
        state.lost_frames++;

        // 前10次丢失都打印
        if(lost_log_count < 10)
        {
            printf("[TRACK] Lost #%d (total_lost=%lu)\r\n", lost_log_count, state.lost_frames);
            lost_log_count++;
        }

        // 丢失超过30帧（约2秒@15fps）后启动回中
        if(state.lost_frames >= 30 && !state.returning_to_center)
        {
            state.returning_to_center = 1;
            printf("[TRACK] Lost target for 2 seconds, starting return to center...\r\n");
        }

        // 回中逻辑：平滑移动舵机回到中心点（90度）
        if(state.returning_to_center)
        {
            float center_pan = 90.0f;
            float center_tilt = 90.0f;
            float new_pan, new_tilt;
            float pan_delta, tilt_delta;
            float return_speed = 0.25f;  // 回中速度（每帧移动25%的距离，快速回中）

            // 平滑插值到中心点
            new_pan = state.pan_angle * (1.0f - return_speed) + center_pan * return_speed;
            new_tilt = state.tilt_angle * (1.0f - return_speed) + center_tilt * return_speed;

            // 计算角度变化量
            pan_delta = fabsf(new_pan - state.pan_angle);
            tilt_delta = fabsf(new_tilt - state.tilt_angle);

            // 仅当角度变化超过0.1度时才移动（避免微小抖动）
            if(pan_delta >= 0.1f || tilt_delta >= 0.1f)
            {
                state.pan_angle = new_pan;
                state.tilt_angle = new_tilt;

                // 控制舵机
                Servo_SetAngle(config.pan_channel, state.pan_angle);
                Servo_SetAngle(config.tilt_channel, state.tilt_angle);

                // 每30帧输出一次回中进度
                if(state.lost_frames % 30 == 0)
                {
                    printf("[TRACK] Returning to center... Pan=%.1f Tilt=%.1f\r\n",
                           state.pan_angle, state.tilt_angle);
                }
            }
            else
            {
                // 已经到达中心点，停止回中
                if(state.lost_frames % 30 == 0)
                {
                    printf("[TRACK] Centered at (%.1f, %.1f), waiting for target...\r\n",
                           state.pan_angle, state.tilt_angle);
                }
            }
        }

        state.tracking = 0;
    }

    debug_counter++;
}

// ==================== 手动控制 ====================
void ServoTrack_ManualControl(float pan_delta, float tilt_delta)
{
    // 增量控制
    state.pan_angle += pan_delta;
    state.tilt_angle += tilt_delta;

    // 限幅
    limit_angle(&state.pan_angle);
    limit_angle(&state.tilt_angle);

    // 控制舵机
    Servo_SetAngle(config.pan_channel, state.pan_angle);
    Servo_SetAngle(config.tilt_channel, state.tilt_angle);

    printf("[TRACK] Manual: Pan=%.1f度 Tilt=%.1f度\r\n",
           state.pan_angle, state.tilt_angle);
}

void ServoTrack_SetAngles(float pan_angle, float tilt_angle)
{
    // 绝对控制
    state.pan_angle = pan_angle;
    state.tilt_angle = tilt_angle;

    // 限幅
    limit_angle(&state.pan_angle);
    limit_angle(&state.tilt_angle);

    // 控制舵机
    Servo_SetAngle(config.pan_channel, state.pan_angle);
    Servo_SetAngle(config.tilt_channel, state.tilt_angle);
}

// ==================== 参数设置 ====================
void ServoTrack_SetSmoothFactor(float factor)
{
    if(factor < 0.0f) factor = 0.0f;
    if(factor > 1.0f) factor = 1.0f;
    config.smooth_factor = factor;
    printf("[TRACK] Smooth factor set to %.2f\r\n", factor);
}

void ServoTrack_SetDeadZone(uint16_t zone)
{
    config.dead_zone = zone;
    printf("[TRACK] Dead zone set to %d pixels\r\n", zone);
}

void ServoTrack_SetMinAngleChange(float threshold)
{
    if(threshold < 0.0f) threshold = 0.0f;
    config.min_angle_change = threshold;
    printf("[TRACK] Min angle change set to %.1f度\r\n", threshold);
}

// ==================== 状态获取 ====================
float ServoTrack_GetPanAngle(void)
{
    return state.pan_angle;
}

float ServoTrack_GetTiltAngle(void)
{
    return state.tilt_angle;
}

servo_track_state_t* ServoTrack_GetState(void)
{
    return &state;
}

// ==================== 重置功能 ====================
void ServoTrack_Reset(void)
{
    state.pan_angle = 90.0f;
    state.tilt_angle = 90.0f;
    state.tracking = 0;
    state.lost_frames = 0;

    Servo_SetAngle(config.pan_channel, state.pan_angle);
    Servo_SetAngle(config.tilt_channel, state.tilt_angle);

    printf("[TRACK] Reset to center (90度, 90度)\r\n");
}

// ==================== 调试输出 ====================
void ServoTrack_PrintDebugInfo(void)
{
    printf("\r\n========== Servo Track Debug Info ==========\r\n");
    printf("Mode: %s\r\n", current_mode == SERVO_MODE_MANUAL ? "MANUAL" : "AUTO_TRACK");
    printf("Image Size: %dx%d\r\n", config.img_width, config.img_height);
    printf("Servo Channels: Pan=CH%d, Tilt=CH%d\r\n", config.pan_channel, config.tilt_channel);
    printf("Current Angles: Pan=%.1f度, Tilt=%.1f度\r\n", state.pan_angle, state.tilt_angle);
    printf("Target Coord: (%d, %d)\r\n", state.target_x, state.target_y);
    printf("Tracking: %s\r\n", state.tracking ? "YES" : "NO");
    printf("Lost Frames: %lu\r\n", state.lost_frames);
    printf("Smooth Factor: %.2f\r\n", config.smooth_factor);
    printf("Dead Zone: %d pixels\r\n", config.dead_zone);
    printf("==========================================\r\n\r\n");
}

// ==================== 内部函数实现 ====================

/**
 * @brief  坐标转换为角度（调试版本：全范围0-180度线性映射）
 * @param  x: 光斑X坐标 (0~img_width, 通常320)
 * @param  y: 光斑Y坐标 (0~img_height, 通常240)
 * @param  pan: 输出水平角度指针
 * @param  tilt: 输出垂直角度指针
 * @note   调试映射关系（标准线性）：
 *         水平(Pan)：  x=0 → 0度(左),    x=160 → 90度, x=320 → 180度(右)
 *         垂直(Tilt)： y=0 → 0度(上抬), y=120 → 90度, y=240 → 180度(低头)
 */
static void coord_to_angles(uint16_t x, uint16_t y, float *pan, float *tilt)
{
    float center_x;
    float center_y;

    center_x = config.img_width / 2.0f;   // 160
    center_y = config.img_height / 2.0f;  // 120

    // 水平角度：标准映射，全范围0-180度
    // x=0→0度, x=160→90度, x=320→180度
    *pan = 90.0f + (x - center_x) * 90.0f / center_x;

    // 垂直角度：限制范围20-160度（扩大活动范围，改善上下追踪）
    // y=0→20度(抬头), y=120→90度, y=240→160度(低头)
    *tilt = 90.0f + (y - center_y) * 70.0f / center_y;

    // 轴反向（如机械安装方向与角度定义相反）
    if(pan_invert_flag)  { *pan  = 180.0f - *pan;  }
    if(tilt_invert_flag) { *tilt = 180.0f - *tilt; }
}

/**
 * @brief  限制角度范围 0~180度
 * @param  angle: 角度指针
 */
static void limit_angle(float *angle)
{
    if(*angle < 0.0f) *angle = 0.0f;
    if(*angle > 180.0f) *angle = 180.0f;
}

// 轴反向配置（适配机械安装方向差异）
void ServoTrack_SetAxisInvert(uint8_t pan_invert, uint8_t tilt_invert)
{
    pan_invert_flag  = (pan_invert != 0) ? 1 : 0;
    tilt_invert_flag = (tilt_invert != 0) ? 1 : 0;
    printf("[TRACK] Axis invert: PAN=%d TILT=%d\r\n", pan_invert_flag, tilt_invert_flag);
}

/**
 * @brief  判断坐标是否在中心死区内
 * @param  x: X坐标
 * @param  y: Y坐标
 * @retval 1:在死区内, 0:不在死区内
 */
static uint8_t in_dead_zone(uint16_t x, uint16_t y)
{
    float center_x = config.img_width / 2.0f;   // 160
    float center_y = config.img_height / 2.0f;  // 120（调试用全范围）

    float dx = x - center_x;
    float dy = y - center_y;
    float distance = sqrtf(dx * dx + dy * dy);

    return (distance <= config.dead_zone) ? 1 : 0;
}

