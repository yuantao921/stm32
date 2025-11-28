/**
 ****************************************************************************************************
 * @file        spot_detect.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-xx
 * @brief       光斑检测接口 - STM32F407适配版本
 * @note        此文件为接口占位，实际光斑检测需要在PC端或后续实现
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

/**
 ****************************************************************************************************
 * @file        spot_detect.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-xx
 * @brief       光斑检测接口 - STM32F407适配版本
 * @note        支持RGB565模式检测，JPEG模式需要PC端或后续实现
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

#ifndef __SPOT_DETECT_H
#define __SPOT_DETECT_H

#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/
/* 检测结果结构体 */

typedef struct {
    uint16_t x;              /* 光斑中心X坐标 */
    uint16_t y;              /* 光斑中心Y坐标 */
    uint8_t found;           /* 是否检测到光斑 */
    uint16_t intensity;      /* 光斑强度(检测到的亮点数量) */
} spot_result_t;

/******************************************************************************************/
/* 光斑检测接口定义 */

/**
 * @brief  初始化光斑检测模块
 * @param  threshold: 亮度阈值 (0-255，建议240)
 * @retval None
 */
void SPOT_Detect_Init(uint16_t threshold);

/**
 * @brief  重置追踪状态
 * @retval None
 */
void SPOT_Detect_Reset(void);

/**
 * @brief  检查是否检测到光斑
 * @retval 1:检测到光斑, 0:未检测到光斑
 */
uint8_t SPOT_Detect_IsFound(void);

/**
 * @brief  获取光斑中心坐标
 * @param  x: 光斑X坐标指针 (0~img_width)
 * @param  y: 光斑Y坐标指针 (0~img_height)
 * @retval None
 */
void SPOT_Detect_GetCenter(uint16_t *x, uint16_t *y);

/**
 * @brief  获取完整检测结果
 * @retval 检测结果结构体指针
 */
spot_result_t* SPOT_Detect_GetResult(void);

/**
 * @brief  设置亮度阈值
 * @param  threshold: 亮度阈值 (0-255，建议240)
 * @retval None
 */
void SPOT_Detect_SetBrightThreshold(uint16_t threshold);

/**
 * @brief  处理RGB565帧（质心法-推荐）
 * @param  rgb_buf: RGB565数据缓冲区指针
 * @param  img_w: 图像宽度 (像素)
 * @param  img_h: 图像高度 (像素)
 * @retval None
 * @note   使用质心法检测，局部窗口抗光晕干扰，推荐使用
 */
void SPOT_Detect_RGB565_Centroid(uint16_t *rgb_buf, uint16_t img_w, uint16_t img_h);

/**
 * @brief  处理RGB565帧（滑动窗口法）
 * @param  rgb_buf: RGB565数据缓冲区指针
 * @param  img_w: 图像宽度 (像素)
 * @param  img_h: 图像高度 (像素)
 * @retval None
 * @note   使用滑动窗口法检测，适合多光源场景
 */
void SPOT_Detect_RGB565(uint16_t *rgb_buf, uint16_t img_w, uint16_t img_h);

/**
 * @brief  处理JPEG帧（简化采样检测）
 * @param  jpeg_buf: JPEG数据缓冲区指针
 * @param  jpeg_len: JPEG数据长度
 * @param  img_w: 图像宽度 (像素)
 * @param  img_h: 图像高度 (像素)
 * @retval None
 * @note   通过采样JPEG数据估算光斑位置，准确度不如RGB565检测
 *         建议：如需高精度检测，请切换到RGB565模式或使用PC端检测
 */
void SPOT_Detect_Process(uint8_t *jpeg_buf, uint32_t jpeg_len, uint16_t img_w, uint16_t img_h);

/**
 * @brief  JPEG解码后检测（需要外部解码库支持）
 * @param  rgb_buf: RGB565数据缓冲区指针（解码后的数据）
 * @param  img_w: 图像宽度 (像素)
 * @param  img_h: 图像高度 (像素)
 * @retval None
 * @note   如果JPEG已解码为RGB565，可以直接调用此函数进行检测
 *         使用方法：
 *         1. 使用JPEG解码库（如TJpgDec）解码JPEG为RGB565
 *         2. 调用此函数进行检测
 */
void SPOT_Detect_Process_Decoded(uint16_t *rgb_buf, uint16_t img_w, uint16_t img_h);

/**
 * @brief  设置光斑坐标（用于测试或从串口接收）
 * @param  x: 光斑X坐标 (0~img_width)
 * @param  y: 光斑Y坐标 (0~img_height)
 * @retval None
 * @note   此函数用于手动设置光斑坐标，便于测试追踪功能
 */
void SPOT_Detect_SetCenter(uint16_t x, uint16_t y);

/**
 * @brief  设置光斑检测状态
 * @param  found: 1=检测到光斑, 0=未检测到光斑
 * @retval None
 * @note   此函数用于手动设置检测状态，便于测试追踪功能
 */
void SPOT_Detect_SetFound(uint8_t found);

#endif

