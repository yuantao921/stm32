/**
 ****************************************************************************************************
 * @file        Servo.c
 * @author      改写自STM32F103版本
 * @version     V1.0
 * @date        2024
 * @brief       舵机控制驱动代码 (DS3218舵机) - STM32F407 HAL库版本
 * @note
 *              实验平台:正点原子 探索者 F407开发板
 *              - PA0控制左右舵机 (TIM2_CH1)
 *              - PA1控制上下舵机 (TIM2_CH2)
 *              
 *              时钟计算说明 (F407):
 *              - 系统时钟(HCLK) = 168MHz
 *              - APB1分频系数 = 4, 所以APB1时钟(PCLK1) = 168MHz / 4 = 42MHz
 *              - STM32F4定时器时钟规则:
 *                * 当APB1预分频系数 > 1时, 定时器时钟(TIMxCLK) = PCLK1 × 2 = 42MHz × 2 = 84MHz
 *                * 当APB1预分频系数 = 1时, 定时器时钟(TIMxCLK) = PCLK1
 *              - 要实现1MHz计数频率: psc = 84MHz / 1MHz - 1 = 83
 ****************************************************************************************************
 */

#include "./BSP/Servo/Servo.h"
#include "./SYSTEM/delay/delay.h"

static float servo_center_offset[2] = {0.0f, 0.0f};   // 只支持通道0和1
static float servo_angle_min[2] = {0.0f, 0.0f};
static float servo_angle_max[2] = {180.0f, 180.0f};

TIM_HandleTypeDef g_tim2_handler;          /* TIM2定时器句柄 */
TIM_OC_InitTypeDef g_tim2_ch1handler;      /* TIM2通道1句柄 (左右舵机) */
TIM_OC_InitTypeDef g_tim2_ch2handler;      /* TIM2通道2句柄 (上下舵机) */

/**
 * @brief       定时器底层驱动，时钟使能，引脚配置
 * @note        此函数会被HAL_TIM_PWM_Init()调用
 * @param       htim:定时器句柄
 * @retval      无
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef gpio_init_struct;

    if (htim->Instance == SERVO_TIMX)
    {
        SERVO_TIMX_CLK_ENABLE();           /* 使能定时器2 */
        SERVO_GPIO_CLK_ENABLE();           /* 舵机 GPIO 时钟使能 */

        /* 配置PA0 (TIM2_CH1, 左右舵机) */
        gpio_init_struct.Pin = SERVO_LEFT_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        gpio_init_struct.Alternate = SERVO_GPIO_AF;
        HAL_GPIO_Init(SERVO_GPIO_PORT, &gpio_init_struct);

        /* 配置PA1 (TIM2_CH2, 上下舵机) */
        gpio_init_struct.Pin = SERVO_UP_GPIO_PIN;
        HAL_GPIO_Init(SERVO_GPIO_PORT, &gpio_init_struct);
    }
}

/**
 * @brief       初始化 TIM2 双路舵机PWM
 * @note
 *              配置为50Hz频率 (20ms周期), 目标计数频率1MHz
 *              F407时钟: 168MHz系统时钟 → 预分频167 → 1MHz计数频率
 * @param       无
 * @retval      无
 */
void Servo_TIM2_Init(void)
{
    uint32_t tim2_freq;      /* TIM2定时器时钟频率 (Hz) */
    uint32_t pclk1_freq;     /* APB1时钟频率 (Hz) */
    uint32_t target_freq = 1000000;  /* 目标计数频率: 1MHz */
    uint16_t arr = PWM_PERIOD;       /* 自动重装载值 (20ms @ 1MHz, ARR=19999) */
    uint16_t psc;
    
    /* 获取APB1时钟频率 */
    pclk1_freq = HAL_RCC_GetPCLK1Freq();
    
    /* 计算TIM2实际时钟频率 —— F4系列定时器时钟逻辑 */
    {
        RCC_ClkInitTypeDef clk_init_struct;
        uint32_t flash_latency;
        HAL_RCC_GetClockConfig(&clk_init_struct, &flash_latency);

        if (clk_init_struct.APB1CLKDivider == RCC_HCLK_DIV1)
            tim2_freq = pclk1_freq;       // APB1=1 → tim = pclk1
        else
            tim2_freq = pclk1_freq * 2;   // APB1分频>1时，定时器时钟=PCLK1×2（正确逻辑）
    }
    
    /* 计算预分频器值, 使计数频率为1MHz */
    psc = (tim2_freq / target_freq) - 1;

    g_tim2_handler.Instance = SERVO_TIMX;                  /* 定时器2 */
    g_tim2_handler.Init.Prescaler = psc;                   /* 定时器分频 */
    g_tim2_handler.Init.CounterMode = TIM_COUNTERMODE_UP;  /* 向上计数模式 */
    g_tim2_handler.Init.Period = arr;                      /* 自动重装载值 */
    g_tim2_handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; /* 时钟分频 */
    HAL_TIM_PWM_Init(&g_tim2_handler);                     /* 初始化PWM */

    /* 配置通道1 (左右舵机) */
    g_tim2_ch1handler.OCMode = TIM_OCMODE_PWM1;            /* PWM模式1 */
    g_tim2_ch1handler.Pulse = PWM_MID;                     /* 设置比较值,默认中位 (1.5ms) */
    g_tim2_ch1handler.OCPolarity = TIM_OCPOLARITY_HIGH;    /* 输出比较极性为高 */
    g_tim2_ch1handler.OCFastMode = TIM_OCFAST_DISABLE;     /* 快速模式禁止 */
    HAL_TIM_PWM_ConfigChannel(&g_tim2_handler, &g_tim2_ch1handler, SERVO_LEFT_CHANNEL);

    /* 配置通道2 (上下舵机) */
    g_tim2_ch2handler.OCMode = TIM_OCMODE_PWM1;            /* PWM模式1 */
    g_tim2_ch2handler.Pulse = PWM_MID;                     /* 设置比较值,默认中位 (1.5ms) */
    g_tim2_ch2handler.OCPolarity = TIM_OCPOLARITY_HIGH;    /* 输出比较极性为高 */
    g_tim2_ch2handler.OCFastMode = TIM_OCFAST_DISABLE;     /* 快速模式禁止 */
    HAL_TIM_PWM_ConfigChannel(&g_tim2_handler, &g_tim2_ch2handler, SERVO_UP_CHANNEL);

    /* 启动PWM输出 */
    HAL_TIM_PWM_Start(&g_tim2_handler, SERVO_LEFT_CHANNEL);  /* 开启PWM通道1 */
    HAL_TIM_PWM_Start(&g_tim2_handler, SERVO_UP_CHANNEL);    /* 开启PWM通道2 */

    /* 默认将CH0、CH1映射到物理0°~180° */
    Servo_SetAngleRange(0, 0.0f, 180.0f);
    Servo_SetAngleRange(1, 0.0f, 180.0f);
}

/**
 * @brief       内部函数：设置脉宽（单位：counts，对应us）
 * @param       ch: 通道号 (0=左右舵机, 1=上下舵机)
 * @param       pulse_us: 脉宽值 (500~2500, 对应0.5ms~2.5ms)
 * @retval      无
 */
static void Servo_SetPulse(uint8_t ch, uint16_t pulse_us)
{
    uint32_t channel;
    
    if(pulse_us < PWM_MIN) pulse_us = PWM_MIN;
    if(pulse_us > PWM_MAX) pulse_us = PWM_MAX;

    if(ch == 0)
        channel = SERVO_LEFT_CHANNEL;
    else if(ch == 1)
        channel = SERVO_UP_CHANNEL;
    else
        return;

    /* 设置PWM脉宽 */
    __HAL_TIM_SET_COMPARE(&g_tim2_handler, channel, pulse_us);
}

/**
 * @brief       设置舵机角度
 * @note
 *              将角度转换为对应的PWM脉宽
 *              角度范围: 0~180度
 *              脉宽范围: 0.5ms~2.5ms (500~2500 counts @ 1MHz)
 *
 * @param       ch: 通道号, 0=左右舵机, 1=上下舵机
 * @param       angle: 角度值, 0~180度
 * @retval      无
 */
void Servo_SetAngle(uint8_t ch, float angle)
{
    uint16_t pulse;
    float logical_angle = angle;
    float mapped_angle;
    float min_range;
    float max_range;
    float span;
    float normalized;

    if(ch >= 2)
        return;

    if(logical_angle < 0.0f)   logical_angle = 0.0f;
    if(logical_angle > 180.0f) logical_angle = 180.0f;

    min_range = servo_angle_min[ch];
    max_range = servo_angle_max[ch];
    span = max_range - min_range;
    if(span <= 0.0f)
    {
        span = 180.0f;
        max_range = min_range + span;
    }

    mapped_angle = min_range + span * (logical_angle / 180.0f);
    mapped_angle += servo_center_offset[ch];

    if(mapped_angle < min_range) mapped_angle = min_range;
    if(mapped_angle > max_range) mapped_angle = max_range;

    normalized = (mapped_angle - min_range) / span;
    pulse = (uint16_t)(PWM_MIN + (PWM_MAX - PWM_MIN) * normalized + 0.5f);

    Servo_SetPulse(ch, pulse);
}

/**
 * @brief       设置舵机中位校准偏移
 * @param       ch: 通道号, 0=左右舵机, 1=上下舵机
 * @param       offset: 偏移角度（度），正值向右/上，负值向左/下
 * @retval      无
 */
void Servo_SetCenterOffset(uint8_t ch, float offset)
{
    if(ch < 2) servo_center_offset[ch] = offset;
}

/**
 * @brief       获取舵机中位校准偏移
 * @param       ch: 通道号, 0=左右舵机, 1=上下舵机
 * @retval      偏移角度（度）
 */
float Servo_GetCenterOffset(uint8_t ch)
{
    if(ch < 2) return servo_center_offset[ch];
    return 0.0f;
}

/**
 * @brief       设置舵机物理角度范围
 * @param       ch: 通道号, 0=左右舵机, 1=上下舵机
 * @param       min_angle: 最小物理角度（度）
 * @param       max_angle: 最大物理角度（度）
 * @retval      无
 */
void Servo_SetAngleRange(uint8_t ch, float min_angle, float max_angle)
{
    if(ch < 2 && max_angle > min_angle)
    {
        servo_angle_min[ch] = min_angle;
        servo_angle_max[ch] = max_angle;
    }
}

/**
 * @brief       双路同步自检（非常平滑好看）
 * @note
 *              执行平滑的舵机动作序列，用于开机自检和展示
 *              动作序列：
 *              1. 先回中位90°，等待舵机稳定
 *              2. 两个舵机同时从0°平滑扫描到180°（每10度一步，每步30ms）
 *              3. 两个舵机同时从180°平滑扫描回0°
 *              4. 左右舵机先转到20°，然后上下舵机转到20°
 *              5. 最后回到中位90°完成自检
 *
 * @param       无
 * @retval      无
 */
void Servo_SelfTest_Dual(void)
{
    uint16_t i;

    // 1. 先回中位，避免暴力转动
    Servo_SetAngle(0, 90);
    Servo_SetAngle(1, 90);
    delay_ms(800);

    // 2. 0° → 180°
    for(i = 0; i <= 180; i += 10)
    {
        Servo_SetAngle(0, (float)i);
        Servo_SetAngle(1, (float)i);
        delay_ms(30);
    }
    delay_ms(600);

    // 3. 180° → 0°
    for(i = 180; i > 0; i -= 10)
    {
        Servo_SetAngle(0, (float)i);
        Servo_SetAngle(1, (float)i);
        delay_ms(30);
    }
    Servo_SetAngle(0, 20.0f); 
    delay_ms(300);
    Servo_SetAngle(1, 20.0f); 
    delay_ms(300);

    // 4. 最后回90°中位
    Servo_SetAngle(0, 90);
    Servo_SetAngle(1, 90);
    delay_ms(600);
}

