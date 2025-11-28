/**
 ****************************************************************************************************
 * @file        spot_detect.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-xx
 * @brief       光斑检测接口实现 - STM32F407适配版本
 * @note        支持RGB565模式检测（质心法），JPEG模式需要PC端或后续实现
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

#include "./BSP/SERVO_TRACK/spot_detect.h"
#include "./SYSTEM/usart/usart.h"
#include <math.h>

//////////////////////////////////////////////////////////////////////////////////
// ALIENTEK STM32F407
// 光斑检测模块 - 基于RGB565数据分析
// 原理：RGB565按像素顺序存储，通过分析亮度分布检测光斑位置
//////////////////////////////////////////////////////////////////////////////////

/******************************************************************************************/
/* 宏定义 */

#define CENTROID_SCORE_THRESHOLD 1000000ULL  /* 质心法最少需要的亮度平方和（Score） */
#define LOCAL_WINDOW_SIZE 30                 /* 局部窗口大小（只在最亮点周围计算） */

/******************************************************************************************/
/* 内部变量 */

static spot_result_t spot_result;           /* 检测结果 */
static uint16_t bright_threshold = 240;     /* 强光源阈值（0-255） */
static uint32_t frame_count = 0;            /* 帧计数 */

/* RGB565追踪算法的状态变量 */
static float last_x = 160.0f;
static float last_y = 120.0f;
static float last_vx = 0.0f;
static float last_vy = 0.0f;
static uint8_t init_frames = 0;
static uint8_t lost_count = 0;
static uint8_t extreme_jump_count = 0;

/******************************************************************************************/
/* 内部函数声明 */

static uint8_t rgb565_to_brightness(uint16_t rgb565);

/******************************************************************************************/
/* 接口实现 */

/**
 * @brief  初始化光斑检测模块
 * @param  threshold: 亮度阈值 (0-255，建议240)
 * @retval None
 */
void SPOT_Detect_Init(uint16_t threshold)
{
    bright_threshold = threshold;
    SPOT_Detect_Reset();
}

/**
 * @brief  重置追踪状态
 * @retval None
 */
void SPOT_Detect_Reset(void)
{
    /* 重置检测结果 */
    spot_result.found = 0;
    spot_result.x = 0;
    spot_result.y = 0;
    spot_result.intensity = 0;

    /* 重置帧计数 */
    frame_count = 0;

    /* 重置RGB565追踪算法状态 */
    last_x = 160.0f;
    last_y = 120.0f;
    last_vx = 0.0f;
    last_vy = 0.0f;
    init_frames = 0;
    lost_count = 0;
    extreme_jump_count = 0;
}

/**
 * @brief  检查是否检测到光斑
 * @retval 1:检测到光斑, 0:未检测到光斑
 */
uint8_t SPOT_Detect_IsFound(void)
{
    return spot_result.found;
}

/**
 * @brief  获取光斑中心坐标
 * @param  x: 光斑X坐标指针 (0~img_width)
 * @param  y: 光斑Y坐标指针 (0~img_height)
 * @retval None
 */
void SPOT_Detect_GetCenter(uint16_t *x, uint16_t *y)
{
    if(x != NULL) *x = spot_result.x;
    if(y != NULL) *y = spot_result.y;
}

/**
 * @brief  获取完整检测结果
 * @retval 检测结果结构体指针
 */
spot_result_t* SPOT_Detect_GetResult(void)
{
    return &spot_result;
}

/**
 * @brief  设置亮度阈值
 * @param  threshold: 亮度阈值 (0-255，建议240)
 * @retval None
 */
void SPOT_Detect_SetBrightThreshold(uint16_t threshold)
{
    bright_threshold = threshold;
}

/**
 * @brief  设置光斑坐标（用于测试或从串口接收）
 * @param  x: 光斑X坐标 (0~img_width)
 * @param  y: 光斑Y坐标 (0~img_height)
 * @retval None
 */
void SPOT_Detect_SetCenter(uint16_t x, uint16_t y)
{
    spot_result.x = x;
    spot_result.y = y;
}

/**
 * @brief  设置光斑检测状态
 * @param  found: 1=检测到光斑, 0=未检测到光斑
 * @retval None
 */
void SPOT_Detect_SetFound(uint8_t found)
{
    spot_result.found = (found != 0) ? 1 : 0;
}

/******************************************************************************************/
/* 内部函数实现 */

/**
 * @brief  RGB565转亮度值函数
 * @param  rgb565: RGB565格式像素值
 * @retval 亮度值 (0-255)
 * @note   RGB565格式: RRRRR GGGGGG BBBBB (16位)
 *         使用亮度公式: Y = 0.299*R + 0.587*G + 0.114*B
 */
static uint8_t rgb565_to_brightness(uint16_t rgb565)
{
    uint8_t r, g, b;
    uint16_t brightness;
    
    /* 提取RGB565的各个分量 */
    r = (rgb565 >> 11) & 0x1F;  /* 提取5位红色 */
    g = (rgb565 >> 5) & 0x3F;   /* 提取6位绿色 */
    b = rgb565 & 0x1F;          /* 提取5位蓝色 */
    
    /* 扩展到8位范围 */
    r = (r << 3) | (r >> 2);  /* 5位扩展到8位 */
    g = (g << 2) | (g >> 4);  /* 6位扩展到8位 */
    b = (b << 3) | (b >> 2);  /* 5位扩展到8位 */
    
    /* 使用整数运算计算亮度 (避免浮点运算) */
    /* Y = 0.299*R + 0.587*G + 0.114*B ≈ (77*R + 150*G + 29*B) / 256 */
    brightness = (77 * r + 150 * g + 29 * b) >> 8;
    
    return (uint8_t)brightness;
}

/******************************************************************************************/
/* RGB565检测算法实现 */

/**
 * @brief  处理RGB565帧（质心法-推荐）
 * @param  rgb_buf: RGB565数据缓冲区指针
 * @param  img_w: 图像宽度 (像素)
 * @param  img_h: 图像高度 (像素)
 * @retval None
 * @note   使用质心法检测，局部窗口抗光晕干扰，推荐使用
 */
void SPOT_Detect_RGB565_Centroid(uint16_t *rgb_buf, uint16_t img_w, uint16_t img_h)
{
    uint16_t x, y;
    uint8_t brightness;
    unsigned long long x_weighted_sum = 0;  /* 使用64位防止溢出 */
    unsigned long long y_weighted_sum = 0;
    unsigned long long total_weight = 0;
    uint32_t pixel_count = 0;
    uint8_t max_brightness = 0;
    uint16_t max_x = 0, max_y = 0;  /* 最亮点位置 */
    uint16_t centroid_x, centroid_y;
    float filtered_x, filtered_y;
    float alpha = 0.75f;  /* 快速响应滤波 */
    uint8_t core_threshold;    /* 核心强光阈值 */
    uint16_t i;

    /* 局部窗口参数 */
    uint16_t win_x_start, win_x_end, win_y_start, win_y_end;

    /* 函数入口诊断（前3帧） */
    if(frame_count < 3)
    {
        printf("\r\n[DEBUG-CENTROID] Frame#%lu Entry: buf=%p, size=%dx%d\r\n",
               frame_count, (void*)rgb_buf, img_w, img_h);
        /* 检查前10个像素的数据 */
        printf("[DEBUG-CENTROID] First 10 pixels: ");
        for(i = 0; i < 10 && i < img_w * img_h; i++)
            printf("%04X ", rgb_buf[i]);
        printf("\r\n");
    }

    /* 第一步：全局扫描找最亮点位置 */
    for(y = 0; y < img_h; y++)
    {
        for(x = 0; x < img_w; x++)
        {
            brightness = rgb565_to_brightness(rgb_buf[y * img_w + x]);
            if(brightness > max_brightness)
            {
                max_brightness = brightness;
                max_x = x;
                max_y = y;
            }
        }
    }

    /* 前3帧输出完整诊断 */
    if(frame_count < 3)
    {
        uint16_t sample_x, sample_y;
        printf("\r\n========== RGB565-CENTROID Frame#%lu Diagnosis ==========\r\n", frame_count);
        printf("[FRAME] Resolution: %dx%d, Threshold=%d\r\n", img_w, img_h, bright_threshold);
        printf("[BRIGHT] Max=%d at (%d,%d)\r\n", max_brightness, max_x, max_y);

        /* 输出像素采样数据（每隔20行，每隔40列） */
        printf("[PIXEL_SAMPLE] Format: (x,y)=RGB565_hex brightness\r\n");
        for(sample_y = 0; sample_y < img_h; sample_y += 20)
        {
            printf("Y=%3d: ", sample_y);
            for(sample_x = 0; sample_x < img_w; sample_x += 40)
            {
                uint16_t pixel = rgb_buf[sample_y * img_w + sample_x];
                uint8_t br = rgb565_to_brightness(pixel);
                printf("(%3d,%3d)=%04X,%3d ", sample_x, sample_y, pixel, br);
            }
            printf("\r\n");
        }
        printf("==================================================\r\n\r\n");
    }

    /* 检查最大亮度是否达到阈值 */
    if(max_brightness < bright_threshold)
    {
        if(frame_count < 3)
            printf("[CENTROID-CHECK1] FAIL: max_brightness=%d < threshold=%d\r\n",
                   max_brightness, bright_threshold);
        spot_result.found = 0;
        spot_result.intensity = 0;
        lost_count++;
        frame_count++;
        return;
    }

    /* 计算局部窗口范围（以最亮点为中心） */
    win_x_start = (max_x > LOCAL_WINDOW_SIZE/2) ? (max_x - LOCAL_WINDOW_SIZE/2) : 0;
    win_x_end = (max_x + LOCAL_WINDOW_SIZE/2 < img_w) ? (max_x + LOCAL_WINDOW_SIZE/2) : img_w;
    win_y_start = (max_y > LOCAL_WINDOW_SIZE/2) ? (max_y - LOCAL_WINDOW_SIZE/2) : 0;
    win_y_end = (max_y + LOCAL_WINDOW_SIZE/2 < img_h) ? (max_y + LOCAL_WINDOW_SIZE/2) : img_h;

    /* 动态核心阈值：取最大亮度的80%（只计算光源核心） */
    core_threshold = max_brightness * 80 / 100;
    if(core_threshold < bright_threshold)
        core_threshold = bright_threshold;

    /* 第二步：只在局部窗口内扫描（忽略远处的光晕和反射） */
    for(y = win_y_start; y < win_y_end; y++)
    {
        for(x = win_x_start; x < win_x_end; x++)
        {
            brightness = rgb565_to_brightness(rgb_buf[y * img_w + x]);

            /* 只统计核心强光像素 */
            if(brightness >= core_threshold)
            {
                uint32_t weight = (uint32_t)brightness * brightness;  /* 亮度平方加权 */
                x_weighted_sum += (unsigned long long)x * weight;
                y_weighted_sum += (unsigned long long)y * weight;
                total_weight += weight;
                pixel_count++;
            }
        }
    }

    /* 第三步：质量检查 */
    /* 检查1：像素数量必须足够（局部窗口内） */
    if(pixel_count < 30 || total_weight == 0)
    {
        if(frame_count < 3)
            printf("[CENTROID-CHECK2] FAIL: pixel_count=%lu (need >=30), max_pos=(%d,%d)\r\n",
                   pixel_count, max_x, max_y);
        spot_result.found = 0;
        spot_result.intensity = 0;
        lost_count++;
        frame_count++;
        return;
    }

    /* 检查2：总权重(Score)必须足够大，过滤环境光 */
    if(total_weight < CENTROID_SCORE_THRESHOLD)
    {
        if(frame_count < 3)
            printf("[CENTROID-CHECK3] FAIL: total_weight=%llu (need >=%llu)\r\n",
                   total_weight, (unsigned long long)CENTROID_SCORE_THRESHOLD);
        spot_result.found = 0;
        spot_result.intensity = 0;
        lost_count++;
        frame_count++;
        return;
    }

    /* 前3帧输出检测成功信息 */
    if(frame_count < 3)
    {
        printf("[CENTROID-DETECT] SUCCESS: max_pos=(%d,%d), pixels=%lu, core_threshold=%d\r\n",
               max_x, max_y, pixel_count, core_threshold);
    }

    /* 第四步：计算质心坐标 */
    centroid_x = (uint16_t)(x_weighted_sum / total_weight);
    centroid_y = (uint16_t)(y_weighted_sum / total_weight);

    /* 修正水平镜像问题（OV2640 RGB565输出是镜像的） */
    centroid_x = img_w - 1 - centroid_x;

    /* 第五步：平滑滤波（快速响应，alpha=0.75） */
    if(init_frames < 10)
    {
        /* 初始化阶段：快速锁定 */
        filtered_x = (float)centroid_x;
        filtered_y = (float)centroid_y;
        last_x = filtered_x;
        last_y = filtered_y;
        init_frames++;
    }
    else
    {
        /* 快速响应滤波（alpha=0.75，保持检测稳定性） */
        filtered_x = last_x * (1.0f - alpha) + centroid_x * alpha;
        filtered_y = last_y * (1.0f - alpha) + centroid_y * alpha;

        /* 更新历史位置 */
        last_x = filtered_x;
        last_y = filtered_y;
    }

    /* 第六步：输出结果 */
    spot_result.x = (uint16_t)filtered_x;
    spot_result.y = (uint16_t)filtered_y;
    spot_result.found = 1;
    spot_result.intensity = pixel_count;
    lost_count = 0;

    frame_count++;
}

/**
 * @brief  处理RGB565帧（滑动窗口法）
 * @param  rgb_buf: RGB565数据缓冲区指针
 * @param  img_w: 图像宽度 (像素)
 * @param  img_h: 图像高度 (像素)
 * @retval None
 * @note   使用滑动窗口法检测，适合多光源场景（简化版本）
 */
void SPOT_Detect_RGB565(uint16_t *rgb_buf, uint16_t img_w, uint16_t img_h)
{
    /* 简化实现：直接调用质心法 */
    SPOT_Detect_RGB565_Centroid(rgb_buf, img_w, img_h);
}

/******************************************************************************************/
/* JPEG检测算法实现 */

/**
 * @brief  简化的JPEG采样检测（通过分析JPEG数据估算光斑位置）
 * @param  jpeg_buf: JPEG数据缓冲区指针
 * @param  jpeg_len: JPEG数据长度
 * @param  img_w: 图像宽度 (像素)
 * @param  img_h: 图像高度 (像素)
 * @retval None
 * @note   这是一个简化实现，通过采样JPEG数据估算亮度分布
 *         准确度不如RGB565检测，但可以在不解码的情况下工作
 *         建议：如需高精度检测，请切换到RGB565模式或使用PC端检测
 */
void SPOT_Detect_Process(uint8_t *jpeg_buf, uint32_t jpeg_len, uint16_t img_w, uint16_t img_h)
{
    uint32_t i;
    uint32_t sample_count = 0;
    uint32_t bright_sample_count = 0;
    uint32_t total_brightness = 0;
    uint32_t weighted_x = 0;
    uint32_t weighted_y = 0;
    uint8_t brightness;
    uint8_t max_brightness = 0;
    uint16_t max_x = 0, max_y = 0;
    uint16_t estimated_x, estimated_y;
    float filtered_x, filtered_y;
    float alpha = 0.7f;  /* 滤波系数 */
    
    /* 采样参数：每隔N个字节采样一次（避免采样JPEG头信息） */
    #define JPEG_SAMPLE_STEP 16      /* 采样步长 */
    #define JPEG_SKIP_HEADER 100     /* 跳过JPEG头信息 */
    #define JPEG_BRIGHT_THRESHOLD 200  /* 亮度阈值 */
    
    /* 检查数据有效性 */
    if(jpeg_buf == NULL || jpeg_len < JPEG_SKIP_HEADER + 100)
    {
        spot_result.found = 0;
        return;
    }
    
    /* 检查JPEG头（FF D8） */
    if(jpeg_buf[0] != 0xFF || jpeg_buf[1] != 0xD8)
    {
        if(frame_count < 3)
            printf("[JPEG-DETECT] Invalid JPEG header\r\n");
        spot_result.found = 0;
        frame_count++;
        return;
    }
    
    /* 采样JPEG数据，估算亮度分布 */
    /* 注意：这是简化方法，JPEG是压缩格式，采样不准确 */
    for(i = JPEG_SKIP_HEADER; i < jpeg_len - 1 && sample_count < 1000; i += JPEG_SAMPLE_STEP)
    {
        /* 跳过JPEG标记（0xFF后跟非0x00的字节） */
        if(jpeg_buf[i] == 0xFF && jpeg_buf[i+1] != 0x00 && jpeg_buf[i+1] != 0xFF)
        {
            i++;  /* 跳过标记字节 */
            continue;
        }
        
        /* 使用字节值作为亮度估算（不准确，但快速） */
        brightness = jpeg_buf[i];
        
        /* 统计亮度信息 */
        if(brightness > max_brightness)
        {
            max_brightness = brightness;
            /* 估算位置（基于采样位置） */
            max_x = (uint16_t)((i * img_w) / jpeg_len);
            max_y = (uint16_t)((i * img_h) / jpeg_len);
        }
        
        /* 统计高亮区域 */
        if(brightness >= JPEG_BRIGHT_THRESHOLD)
        {
            bright_sample_count++;
            total_brightness += brightness;
            /* 加权位置估算 */
            weighted_x += (uint32_t)((i * img_w) / jpeg_len) * brightness;
            weighted_y += (uint32_t)((i * img_h) / jpeg_len) * brightness;
        }
        
        sample_count++;
    }
    
    /* 前3帧输出诊断信息 */
    if(frame_count < 3)
    {
        printf("\r\n========== JPEG-DETECT Frame#%lu Diagnosis ==========\r\n", frame_count);
        printf("[JPEG] Size: %lu bytes, Resolution: %dx%d\r\n", jpeg_len, img_w, img_h);
        printf("[SAMPLE] Samples: %lu, Bright samples: %lu\r\n", sample_count, bright_sample_count);
        printf("[BRIGHT] Max=%d at estimated (%d,%d)\r\n", max_brightness, max_x, max_y);
        printf("==================================================\r\n\r\n");
    }
    
    /* 检查是否检测到强光 */
    if(max_brightness < bright_threshold || bright_sample_count < 10)
    {
        if(frame_count < 3)
            printf("[JPEG-CHECK] FAIL: max_brightness=%d < threshold=%d or samples=%lu < 10\r\n",
                   max_brightness, bright_threshold, bright_sample_count);
        spot_result.found = 0;
        spot_result.intensity = 0;
        lost_count++;
        frame_count++;
        return;
    }
    
    /* 计算加权中心位置 */
    if(total_brightness > 0)
    {
        estimated_x = (uint16_t)(weighted_x / total_brightness);
        estimated_y = (uint16_t)(weighted_y / total_brightness);
    }
    else
    {
        estimated_x = max_x;
        estimated_y = max_y;
    }
    
    /* 限制在图像范围内 */
    if(estimated_x >= img_w) estimated_x = img_w - 1;
    if(estimated_y >= img_h) estimated_y = img_h - 1;
    
    /* 平滑滤波 */
    if(init_frames < 10)
    {
        /* 初始化阶段：快速锁定 */
        filtered_x = (float)estimated_x;
        filtered_y = (float)estimated_y;
        last_x = filtered_x;
        last_y = filtered_y;
        init_frames++;
    }
    else
    {
        /* 平滑滤波 */
        filtered_x = last_x * (1.0f - alpha) + estimated_x * alpha;
        filtered_y = last_y * (1.0f - alpha) + estimated_y * alpha;
        
        /* 更新历史位置 */
        last_x = filtered_x;
        last_y = filtered_y;
    }
    
    /* 输出结果 */
    spot_result.x = (uint16_t)filtered_x;
    spot_result.y = (uint16_t)filtered_y;
    spot_result.found = 1;
    spot_result.intensity = bright_sample_count;
    lost_count = 0;
    
    if(frame_count < 3)
    {
        printf("[JPEG-DETECT] SUCCESS: estimated=(%d,%d), filtered=(%d,%d), samples=%lu\r\n",
               estimated_x, estimated_y, spot_result.x, spot_result.y, bright_sample_count);
    }
    
    frame_count++;
}

/**
 * @brief  JPEG解码后检测（需要外部解码库支持）
 * @param  rgb_buf: RGB565数据缓冲区指针（解码后的数据）
 * @param  img_w: 图像宽度 (像素)
 * @param  img_h: 图像高度 (像素)
 * @retval None
 * @note   如果JPEG已解码为RGB565，可以直接调用此函数
 *         使用方法：
 *         1. 使用JPEG解码库（如TJpgDec）解码JPEG为RGB565
 *         2. 调用此函数进行检测
 */
void SPOT_Detect_Process_Decoded(uint16_t *rgb_buf, uint16_t img_w, uint16_t img_h)
{
    /* 如果JPEG已解码为RGB565，直接使用RGB565检测算法 */
    SPOT_Detect_RGB565_Centroid(rgb_buf, img_w, img_h);
}
