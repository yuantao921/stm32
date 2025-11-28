/**
 ****************************************************************************************************
 * @file        main.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-11-08
 * @brief       摄像头 实验
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

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./USMART/usmart.h"
#include "./BSP/KEY/key.h"
#include "./BSP/TIMER/btim.h"
#include "./BSP/OV2640/ov2640.h"
#include "./BSP/OV2640/sccb.h"
#include "./BSP/DCMI/dcmi.h"
#include "./BSP/USART2/usart2.h"
#include "./BSP/ESP8266/esp8266.h"
#include "./BSP/Servo/Servo.h"
#include "./BSP/SERVO_TRACK/servo_track.h"

uint8_t g_ov_mode = 1;                                      /* bit0: 0, RGB565模式;  1, JPEG模式 */

#define jpeg_buf_size       29*1024                         /* 定义JPEG数据缓存jpeg_buf的大小(*4字节) */
#define jpeg_line_size      1*1024                          /* 定义DMA接收数据时,一行数据的最大值 */

__ALIGNED(4) uint32_t g_jpeg_data_buf[jpeg_buf_size];       /* JPEG数据缓存buf */
__ALIGNED(4) uint32_t g_dcmi_line_buf[2][jpeg_line_size];   /* JPEG数据 DMA双缓存buf */

volatile uint32_t g_jpeg_data_len = 0;                      /* buf中的JPEG有效数据长度 */


/**
 * 0,数据没有采集完;
 * 1,数据采集完了,但是还没处理;
 * 2,数据已经处理完成了,可以开始下一帧接收
 */
volatile uint8_t g_jpeg_data_ok = 0;                        /* JPEG数据采集完成标志 */

/* JPEG尺寸支持列表 */
const uint16_t jpeg_img_size_tbl[][2] =
{
    160, 120,       /* QQVGA */
    176, 144,       /* QCIF */
    320, 240,       /* QVGA */
    400,240,        /* WGVGA */
    352,288,        /* CIF */
    640, 480,       /* VGA */
    800, 600,       /* SVGA */
    1024, 768,      /* XGA */
    1280, 800,      /* WXGA */
    1280, 960,      /* XVGA */
    1440, 900,      /* WXGA+ */
    1280, 1024,     /* SXGA */
    1600, 1200,     /* UXGA */
};

const char *EFFECTS_TBL[7] = {"Normal", "Negative", "B&W", "Redish", "Greenish", "Bluish", "Antique"};     /* 7种特效 */
const char *JPEG_SIZE_TBL[13] = {"QQVGA", "QCIF", "QVGA", "WGVGA", "CIF", "VGA", "SVGA", "XGA", "WXGA", "SVGA", "WXGA+", "SXGA", "UXGA"}; /* JPEG图片 13种尺寸 */

/**
 * @brief       处理JPEG数据
 *   @ntoe      在DCMI_IRQHandler中断服务函数里面被调用
 *              当采集完一帧JPEG数据后,调用此函数,切换JPEG BUF.开始下一帧采集.
 *
 * @param       无
 * @retval      无
 */
void jpeg_data_process(void)
{
    uint16_t i;
    uint16_t rlen;      /* 剩余数据长度 */
    uint32_t *pbuf;
    
    if (g_ov_mode)      /* 只有在JPEG格式下,才需要做处理. */
    {
        if (g_jpeg_data_ok == 0)                    /* jpeg数据还未采集完? */
        {
            __HAL_DMA_DISABLE(&g_dma_dcmi_handle);  /* 关闭DMA */
            while(DMA2_Stream1->CR & 0x01);         /* 等待DMA2_Stream1可配置 */

            rlen = jpeg_line_size - __HAL_DMA_GET_COUNTER(&g_dma_dcmi_handle);    /* 得到剩余长度 */
            pbuf = g_jpeg_data_buf + g_jpeg_data_len;   /* 偏移到有效数据末尾,继续添加 */
            if (DMA2_Stream1->CR & (1 << 19))
            {
                for (i = 0; i < rlen; i++)
                {
                    pbuf[i] = g_dcmi_line_buf[1][i];    /* 读取buf1里面的剩余数据 */
                }
            }
            else
            {
                for (i = 0; i < rlen; i++)
                {
                    pbuf[i] = g_dcmi_line_buf[0][i];    /* 读取buf0里面的剩余数据 */
                }
            }

            g_jpeg_data_len += rlen;    /* 加上剩余长度 */
            g_jpeg_data_ok = 1;         /* 标记JPEG数据采集完成,等待其他函数处理 */
        }
        if (g_jpeg_data_ok == 2)        /* 上一次的jpeg数据已经被处理了 */
        {
            __HAL_DMA_SET_COUNTER(&g_dma_dcmi_handle, jpeg_line_size);   /* 传输长度为jpeg_buf_size*4字节 */
            __HAL_DMA_ENABLE(&g_dma_dcmi_handle);   /* 重新传输 */
            g_jpeg_data_ok = 0;         /* 标记数据未采集 */
            g_jpeg_data_len = 0;        /* 数据重新开始 */
        }
    }

}

/**
 * @brief       JPEG数据接收回调函数
 *   @ntoe      在DMA2_Stream1_IRQHandler中断服务函数里面被调用
 *
 * @param       无
 * @retval      无
 */
void jpeg_dcmi_rx_callback(void)
{
    uint16_t i;
    volatile uint32_t *pbuf;
    pbuf = g_jpeg_data_buf + g_jpeg_data_len;   /* 偏移到有效数据末尾 */

    if (DMA2_Stream1->CR & (1 << 19))           /* buf0已满,正常处理buf1 */
    {
        for (i = 0; i < jpeg_line_size; i++)
        {
            pbuf[i] = g_dcmi_line_buf[0][i];    /* 读取buf0里面的数据 */
        }
        
        g_jpeg_data_len += jpeg_line_size;      /* 偏移 */
    }
    else    /* buf1已满,正常处理buf0 */
    {
        for (i = 0; i < jpeg_line_size; i++)
        {
            pbuf[i] = g_dcmi_line_buf[1][i];    /* 读取buf1里面的数据 */
        }
        
        g_jpeg_data_len += jpeg_line_size;      /* 偏移 */
    }
}

/**
 * @brief       JPEG测试
 *   @ntoe      JPEG数据,通过串口2发送给电脑.
 *
 * @param       无
 * @retval      无
 */
void jpeg_test(void)
{
    uint32_t i, jpgstart, jpglen;
    uint8_t *p;
    uint8_t key, headok = 0;
    
    /* 摄像头参数写死，不再通过按键修改 */
    const uint8_t effect = 0;        /* 特效：Normal */
    const uint8_t saturation = 2;    /* 饱和度：默认值 */
    const uint8_t contrast = 2;      /* 对比度：默认值 */
    const uint8_t size = 2;          /* 分辨率：QVGA 320*240 */
    
    /* 设置摄像头参数 */
    ov2640_contrast(contrast);
    ov2640_color_saturation(saturation);
    ov2640_special_effects(effect);
    
    printf("JPEG Size:%s (Fixed)\r\n", JPEG_SIZE_TBL[size]);
    printf("Contrast:%d, Saturation:%d, Effect:%s (Fixed)\r\n", 
           (signed char)contrast - 2, saturation, EFFECTS_TBL[effect]);
    
    ov2640_jpeg_mode(); /* JPEG模式 */
    dcmi_init();        /* DCMI配置 */
    dcmi_rx_callback = jpeg_dcmi_rx_callback;    /* JPEG接收数据回调函数 */
    dcmi_dma_init((uint32_t)&g_dcmi_line_buf[0], (uint32_t)&g_dcmi_line_buf[1], jpeg_line_size, DMA_MDATAALIGN_WORD, DMA_MINC_ENABLE); /* DCMI DMA配置 */
    ov2640_outsize_set(jpeg_img_size_tbl[size][0], jpeg_img_size_tbl[size][1]);     /* 设置输出尺寸 */
    dcmi_start();       /* 启动传输 */

    while (1)
    {
        /* 调用追踪处理函数（自动模式下会根据光斑位置控制舵机） */
        ServoTrack_Process();
        
        /* 按键处理：模式切换（KEY0） */
        key = key_scan(0);  /* 扫描按键（不支持连按模式，用于检测按键按下事件） */
        if (key == KEY0_PRES)
        {
            /* KEY0: 模式切换（手动 ? 自动） */
            if (ServoTrack_GetMode() == SERVO_MODE_MANUAL)
            {
                ServoTrack_SetMode(SERVO_MODE_AUTO_TRACK);
                printf("Mode: AUTO_TRACK\r\n");
            }
            else
            {
                ServoTrack_SetMode(SERVO_MODE_MANUAL);
                printf("Mode: MANUAL\r\n");
            }
            delay_ms(200);  /* 防抖延时 */
        }
        
        /* 手动模式下的持续按键检测（按下持续转动，松手才停） */
        if (ServoTrack_GetMode() == SERVO_MODE_MANUAL)
        {
            float pan_delta = 0.0f;  /* 水平角度增量 */
            float tilt_delta = 0.0f; /* 垂直角度增量 */
            
            /* 直接读取GPIO状态，不经过key_scan（支持持续按下） */
            /* 使用独立if，支持同时按下多个按键（如同时左右或上下） */
            if (KEY1 == 0)  /* KEY1按下：向左转 */
            {
                pan_delta -= 10.0f;
            }
            if (KEY2 == 0)  /* KEY2按下：向右转 */
            {
                pan_delta += 10.0f;
            }
            if (WK_UP == 0)  /* KEY3/WKUP按下：向上转（上拉模式：按下为0） */
            {
                tilt_delta -= 10.0f;
            }
            if (KEY4 == 0)  /* KEY4按下：向下转 */
            {
                tilt_delta += 10.0f;
            }
            
            /* 如果有按键按下，执行转动 */
            if (pan_delta != 0.0f || tilt_delta != 0.0f)
            {
                ServoTrack_ManualControl(pan_delta, tilt_delta);
                delay_ms(50);  /* 控制转动速度，50ms转10度 */
            }
        }
        /* 自动模式下的按键功能暂时为空 */
        
        if (g_jpeg_data_ok == 1)    /* 已经采集完一帧图像了 */
        {
            p = (uint8_t *)g_jpeg_data_buf;
            printf("g_jpeg_data_len:%d\r\n", g_jpeg_data_len * 4);  /* 打印数据长度 */
            jpglen = 0;             /* 设置jpg文件大小为0 */
            headok = 0;             /* 清除jpg头标记 */
            
            for (i = 0; i < g_jpeg_data_len * 4; i++)       /* 查找0XFF,0XD8和0XFF,0XD9,获取jpg文件大小 */
            {
                if ((p[i] == 0XFF) && (p[i + 1] == 0XD8))   /* 找到FF D8 */
                {
                    jpgstart = i;
                    headok = 1; /* 标记找到jpg头(FF D8) */
                }

                if ((p[i] == 0XFF) && (p[i + 1] == 0XD9) && headok) /* 找到头以后,再找FF D9 */
                {
                    jpglen = i - jpgstart + 2;
                    break;
                }
            }

            if (jpglen)         /* 正常的jpeg数据 */
            {
                p += jpgstart;  /* 偏移到0XFF,0XD8处 */
                
                /* JPEG光斑检测（简化采样检测） */
                /* 注意：这是简化方法，准确度不如RGB565检测 */
                /* 如需高精度检测，建议切换到RGB565模式 */
                SPOT_Detect_Process(p, jpglen, jpeg_img_size_tbl[size][0], jpeg_img_size_tbl[size][1]);

                /* 发送JPEG数据包头 (用于标识JPEG开始，方便接收端识别) */
                /* 格式: 0xFF 0xD8 0xFF 0xD9 (JPEG标准头尾，但这里只作为标识) */
                /* 实际发送时，先发送长度信息(4字节，大端序) */
                uint32_t jpeg_size = jpglen;
                uint8_t size_buf[4];
                size_buf[0] = (jpeg_size >> 24) & 0xFF;
                size_buf[1] = (jpeg_size >> 16) & 0xFF;
                size_buf[2] = (jpeg_size >> 8) & 0xFF;
                size_buf[3] = jpeg_size & 0xFF;
                
                /* 发送长度信息 */
                for (i = 0; i < 4; i++)
                {
                    USART2->DR = size_buf[i];
                    while ((USART2->SR & 0X40) == 0);
                }

                /* 发送JPEG数据 */
                for (i = 0; i < jpglen; i++)    /* 发送整个jpg文件 */
                {
                    USART2->DR = p[i];

                    while ((USART2->SR & 0X40) == 0);   /* 循环发送,直到发送完毕 */

                    /* 发送过程中也检测按键（用于中断发送） */
                    if (KEY0 == 0 || KEY1 == 0 || KEY2 == 0 || WK_UP == 0 || KEY4 == 0)
                    {
                        break;
                    }
                }
                
                /* 发送结束标识 (可选，用于标识一帧结束) */
                if (i >= jpglen)  /* 完整发送完成 */
                {
                    USART2->DR = 0xFF;
                    while ((USART2->SR & 0X40) == 0);
                    USART2->DR = 0xD9;
                    while ((USART2->SR & 0X40) == 0);
                }
            }        
            g_jpeg_data_ok = 2;     /* 标记jpeg数据处理完了,可以让DMA去采集下一帧了. */
        }
    }
}
//以下为RGB565模式功能，暂时不用 注释掉
/**
 * @brief       RGB565测试
 *   @ntoe      RGB数据直接显示在LCD上面
 *
 * @param       无
 * @retval      无
 */
/*
void rgb565_test(void)
{
    uint8_t key;
    uint8_t effect = 0, saturation = 3, contrast = 2;
    uint8_t scale = 1;          // 默认是全尺寸缩放 
    uint8_t msgbuf[15];         //消息缓存区 

    lcd_clear(WHITE);
    lcd_show_string(30, 50, 200, 16, 16, "STM32", RED);
    lcd_show_string(30, 70, 200, 16, 16, "OV2640 RGB565 Mode", RED);
    lcd_show_string(30, 100, 200, 16, 16, "KEY0:Contrast", RED);            // 对比度 
    lcd_show_string(30, 120, 200, 16, 16, "KEY1:Saturation", RED);          // 执行自动对焦 
    lcd_show_string(30, 140, 200, 16, 16, "KEY2:Effects", RED);             // 特效 
    lcd_show_string(30, 160, 200, 16, 16, "KEY_UP:FullSize/Scale", RED);    // 1:1尺寸(显示真实尺寸)/全尺寸缩放 

    ov2640_rgb565_mode();       //RGB565模式 
    dcmi_init();                // DCMI配置 
    dcmi_dma_init((uint32_t)&LCD->LCD_RAM, 0, 1, DMA_MDATAALIGN_HALFWORD, DMA_MINC_DISABLE); // DCMI DMA配置,MCU屏,竖屏
    ov2640_outsize_set(lcddev.width, lcddev.height);    //满屏缩放显示
    dcmi_start();               //启动传输
    
    while (1)
    {
        key = key_scan(0);

        if (key)
        {
            dcmi_stop();        //停止显示 
            switch (key)
            {
                case KEY0_PRES: //对比度设置 
                    contrast++;
                    if (contrast > 4)contrast = 0;
                    ov2640_contrast(contrast);
                    printf((char*)msgbuf, "Contrast:%d", (signed char)contrast - 2);
                    break;

                case KEY1_PRES: //饱和度设置 
                    saturation++;
                    if (saturation > 4)saturation = 0;
                    ov2640_color_saturation(saturation);    //饱和度设置 
                    printf((char *)msgbuf, "Saturation:%d", (signed char)saturation - 2);
                    break;
                
                case KEY2_PRES: // 特效设置 
                    effect++;
                    if (effect > 6)effect = 0;
                    ov2640_special_effects(effect);         //设置特效 
                    printf((char *)msgbuf, "%s", EFFECTS_TBL[effect]);
                    break;

                case WKUP_PRES:  //1:1尺寸(显示真实尺寸)/缩放 
                    scale = !scale;

                    if (scale == 0)
                    {
                        ov2640_image_win_set((1600 - lcddev.width) / 2, (1200 - lcddev.height) / 2, lcddev.width, lcddev.height);
                        ov2640_outsize_set(lcddev.width, lcddev.height);
                        printf((char *)msgbuf, "Full Size 1:1");
                    }
                    else
                    {
                        ov2640_image_win_set(0, 0, 1600, 1200);
                        ov2640_outsize_set(lcddev.width, lcddev.height);
                        printf((char *)msgbuf, "Scale");
                    }
                    
                    break;

                default : break;
            }
            lcd_show_string(30, 50, 210, 16, 16, (char*)msgbuf, RED);   //显示提示内容 
            delay_ms(800);
            dcmi_start();   //重新开始传输 
        }
        delay_ms(10);
    }
}
*/
int main(void)
{

    HAL_Init();                             /* 初始化HAL库 */
    sys_stm32_clock_init(336, 8, 2, 7);     /* 设置时钟,168Mhz */
    delay_init(168);                        /* 延时初始化 */
    usart_init(115200);                     /* 串口初始化为115200 */
    usmart_dev.init(84);                    /* 初始化USMART */
    /* 注意: 如果使用ESP8266，波特率改为115200；如果直接连接电脑，可以使用921600 */
    usart2_init(115200);                    /* 初始化串口2波特率为115200 (ESP8266推荐波特率) */
    servo_init();        // 初始化舵机
    servo_self_check();  // 执行开机自检炫酷动作
    
    /* 初始化舵机追踪模块 */
    ServoTrack_Init(0, 1, 320, 240);  // 通道0(水平), 通道1(垂直), 320x240图像
    ServoTrack_SetSmoothFactor(0.3f);  // 设置平滑系数
    ServoTrack_SetDeadZone(10);       // 设置死区半径
    ServoTrack_SetMode(SERVO_MODE_MANUAL);  // 默认手动模式
    
	esp8266_init();
    led_init();                             /* 初始化LED */
    key_init();                             /* 初始化按键 */
    btim_timx_int_init(10000 - 1, 8400 - 1);/* 10KHz计数频率，1秒中断一次 */

    while (ov2640_init())       /* 初始化OV2640 */
    {
        delay_ms(200);
        delay_ms(200);
        LED0_TOGGLE();
    }

    //ov2640_flash_intctrl();     /* 闪光灯控制 */
    
    while (1)
    {
		if (g_ov_mode == 1)
    {
        jpeg_test();
    }
/*
        key = key_scan(0);

        if (key == KEY0_PRES)
        {
            g_ov_mode = 0;      //RGB565模式 
            break;
        }
        else if (key == KEY1_PRES)
        {
            g_ov_mode = 1;      //JPEG模式 
            break;
        }

        delay_ms(5);
    }

    if (g_ov_mode == 1)
    {
        // JPEG模式：初始化ESP8266进行无线传输 
        if (esp8266_init() == 0)
        {
            delay_ms(1000);
        }
        else
        {
            delay_ms(2000);
        }
        jpeg_test();
    }
    else
    {
        rgb565_test();
    }
*/
	
	}
}

