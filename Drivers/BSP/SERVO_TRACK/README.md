# 舵机光斑追踪模块 - STM32F407适配版

## ? 模块说明

该模块实现舵机自动追踪光斑功能，支持手动/自动模式切换，采用平滑滤波算法避免舵机抖动。

## ? 文件结构

```
SERVO_TRACK/
├── servo_track.h    - 头文件，定义API接口
├── servo_track.c    - 实现文件，核心算法
├── spot_detect.h    - 光斑检测接口头文件
├── spot_detect.c    - 光斑检测接口实现（占位版本）
└── README.md        - 本说明文档
```

## ?? 功能特性

### 1. 双模式控制
- **手动模式 (MANUAL)**: 通过按键控制舵机角度
- **自动模式 (AUTO_TRACK)**: 根据光斑坐标自动追踪

### 2. 平滑追踪算法
- 采用一阶低通滤波，避免舵机抖动
- 可调节平滑系数 (0~1)，默认 0.3
- 平滑系数越小，运动越平滑但响应越慢

### 3. 中心死区功能
- 光斑在图像中心区域时不移动舵机
- 避免微小抖动导致舵机频繁调整
- 可调节死区半径，默认 10 像素

### 4. 坐标映射关系

```
图像坐标系 (320x240)          舵机角度系
┌─────────────────┐         ┌─────────────────┐
│ (0,0)    →   (320,0)│         │ 180°   →   0°      │ 水平
│    ↓            ↓     │    →   │    ↓        ↓       │ (Pan)
│ (0,240)  → (320,240)│         │ 180°   →   0°      │
└─────────────────┘         └─────────────────┘
                                ┌─────────────────┐
                                │  0°              │ 垂直
                                │   ↓              │ (Tilt)
                                │ 90° (中心)        │
                                │   ↓              │
                                │ 180°             │
                                └─────────────────┘
```

**映射公式**:
- 水平角度 = 90 + (光斑X - 160) × 90 / 160
- 垂直角度 = 90 + (光斑Y - 120) × 70 / 120

## ? API 接口

### 初始化

```c
// 初始化追踪模块
ServoTrack_Init(uint8_t pan_channel, uint8_t tilt_channel,
                uint16_t img_width, uint16_t img_height);

// 示例：
ServoTrack_Init(0, 1, 320, 240);  // 通道0(水平), 通道1(垂直), 320x240图像
```

### 模式控制

```c
// 设置模式
ServoTrack_SetMode(SERVO_MODE_MANUAL);       // 切换到手动模式
ServoTrack_SetMode(SERVO_MODE_AUTO_TRACK);   // 切换到自动模式

// 获取当前模式
servo_track_mode_t mode = ServoTrack_GetMode();
```

### 追踪处理

```c
// 主循环调用（自动模式下追踪光斑）
ServoTrack_Process();
```

### 手动控制

```c
// 增量控制（相对当前角度）
ServoTrack_ManualControl(-30.0f, 0.0f);   // 水平左转30度
ServoTrack_ManualControl(30.0f, 0.0f);    // 水平右转30度
ServoTrack_ManualControl(0.0f, 30.0f);   // 垂直下转30度

// 绝对控制
ServoTrack_SetAngles(90.0f, 90.0f);       // 设置到中心
```

### 参数设置

```c
// 设置平滑系数 (0~1)
ServoTrack_SetSmoothFactor(0.3f);   // 0.3为默认值，建议范围0.2~0.5

// 设置死区半径 (像素)
ServoTrack_SetDeadZone(10);         // 10像素为默认值，建议5~15

// 设置最小角度变化阈值 (度)
ServoTrack_SetMinAngleChange(1.0f); // 1度为默认值，建议0.5~2.0
```

### 状态获取

```c
// 获取当前角度
float pan = ServoTrack_GetPanAngle();     // 水平角度 0~180
float tilt = ServoTrack_GetTiltAngle();   // 垂直角度 0~180

// 获取完整状态
servo_track_state_t* state = ServoTrack_GetState();
printf("Pan: %.1f, Tilt: %.1f, Tracking: %d\n",
       state->pan_angle, state->tilt_angle, state->tracking);
```

### 辅助功能

```c
// 重置到中心
ServoTrack_Reset();

// 打印调试信息
ServoTrack_PrintDebugInfo();

// 设置轴反向（适配机械安装方向差异）
ServoTrack_SetAxisInvert(0, 0);  // PAN不反转, TILT不反转
```

## ? 按键控制

| 按键 | 功能 | 说明 |
|------|------|------|
| KEY0 | 水平左转30° | 仅手动模式有效 |
| KEY1 | 水平右转30° | 仅手动模式有效 |
| KEY2 | 模式切换 | 手动 ? 自动 |
| WKUP | 垂直下转30° | 仅手动模式有效 |

**注意**: 在自动模式下，KEY0/KEY1/WKUP 保留用于摄像头设置功能。

## ? 使用流程

### 1. 系统启动

```c
// main.c 中的初始化代码
servo_init();        // 初始化舵机
servo_self_check();  // 执行开机自检

// 初始化追踪模块
ServoTrack_Init(0, 1, 320, 240);  // 通道0(水平), 通道1(垂直), 320x240图像
ServoTrack_SetSmoothFactor(0.3f);
ServoTrack_SetDeadZone(10);
ServoTrack_SetMode(SERVO_MODE_MANUAL);  // 默认手动模式
```

### 2. 主循环

#### JPEG模式（当前使用）

```c
while(1) {
    // 追踪处理（自动模式下追踪光斑）
    ServoTrack_Process();
    
    // 摄像头数据处理
    if (g_jpeg_data_ok == 1) {
        // ... JPEG数据处理 ...
        // 注意：JPEG模式需要PC端检测或手动设置坐标
    }
    
    // 按键处理
    key = key_scan(0);
    // ... 按键逻辑 ...
}
```

#### RGB565模式（推荐用于实时检测）

```c
// 切换到RGB565模式
ov2640_rgb565_mode();
dcmi_init();
dcmi_dma_init((uint32_t)rgb_buf, 0, img_size, DMA_MDATAALIGN_HALFWORD, DMA_MINC_ENABLE);
ov2640_outsize_set(320, 240);
dcmi_start();

while(1) {
    // 每帧检测光斑
    if(frame_ready) {
        SPOT_Detect_RGB565_Centroid(rgb_buf, 320, 240);
    }
    
    // 追踪处理（自动模式下追踪光斑）
    ServoTrack_Process();
    
    // 按键处理
    key = key_scan(0);
    // ... 按键逻辑 ...
}
```

### 3. 测试步骤

#### 手动模式测试
1. 上电后默认为手动模式
2. 按 KEY0/KEY1 测试水平舵机
3. 按 WKUP 测试垂直舵机
4. 确认舵机响应正常

#### 自动模式测试
1. 按 KEY2 切换到自动模式
2. 使用 `SPOT_Detect_SetCenter()` 设置测试坐标
3. 使用 `SPOT_Detect_SetFound(1)` 设置检测状态
4. 观察舵机是否追踪到目标位置

## ? 光斑检测接口说明

`spot_detect` 模块已实现 **RGB565模式检测**（质心法），支持实时光斑检测。

### 接口函数

```c
// 初始化光斑检测模块
void SPOT_Detect_Init(uint16_t threshold);  // threshold建议240

// 重置追踪状态
void SPOT_Detect_Reset(void);

// 检查是否检测到光斑
uint8_t SPOT_Detect_IsFound(void);

// 获取光斑中心坐标
void SPOT_Detect_GetCenter(uint16_t *x, uint16_t *y);

// 获取完整检测结果
spot_result_t* SPOT_Detect_GetResult(void);

// 设置亮度阈值
void SPOT_Detect_SetBrightThreshold(uint16_t threshold);

// RGB565模式检测（质心法-推荐）
void SPOT_Detect_RGB565_Centroid(uint16_t *rgb_buf, uint16_t img_w, uint16_t img_h);

// RGB565模式检测（滑动窗口法）
void SPOT_Detect_RGB565(uint16_t *rgb_buf, uint16_t img_w, uint16_t img_h);

// JPEG模式检测（占位实现）
void SPOT_Detect_Process(uint8_t *jpeg_buf, uint32_t jpeg_len, uint16_t img_w, uint16_t img_h);

// 手动设置光斑坐标（用于测试）
void SPOT_Detect_SetCenter(uint16_t x, uint16_t y);

// 手动设置检测状态（用于测试）
void SPOT_Detect_SetFound(uint8_t found);
```

### RGB565模式使用（推荐）

如果使用RGB565模式，可以直接在STM32端检测光斑：

```c
// 1. 初始化检测模块
SPOT_Detect_Init(240);  // 亮度阈值240

// 2. 在RGB565模式下，每帧调用检测函数
// 假设rgb_buf是RGB565数据缓冲区，320x240分辨率
SPOT_Detect_RGB565_Centroid(rgb_buf, 320, 240);

// 3. 检查检测结果
if(SPOT_Detect_IsFound())
{
    uint16_t x, y;
    SPOT_Detect_GetCenter(&x, &y);
    printf("Spot detected at (%d, %d)\r\n", x, y);
}
```

### JPEG模式使用

JPEG模式已实现**简化采样检测**，通过采样JPEG数据估算光斑位置：

```c
// 1. 初始化检测模块
SPOT_Detect_Init(240);  // 亮度阈值240

// 2. 在JPEG模式下，每帧调用检测函数
// 假设jpeg_buf是JPEG数据，jpglen是数据长度，320x240分辨率
SPOT_Detect_Process(jpeg_buf, jpglen, 320, 240);

// 3. 检查检测结果
if(SPOT_Detect_IsFound())
{
    uint16_t x, y;
    SPOT_Detect_GetCenter(&x, &y);
    printf("Spot detected at (%d, %d)\r\n", x, y);
}
```

**注意**：
- JPEG采样检测准确度不如RGB565检测（JPEG是压缩格式）
- 如需高精度检测，建议切换到RGB565模式
- 或者使用PC端检测，通过串口发送坐标

### JPEG解码后检测（高级用法）

如果集成了JPEG解码库（如TJpgDec），可以解码后使用RGB565检测：

```c
// 1. 解码JPEG为RGB565（需要外部解码库）
// decode_jpeg_to_rgb565(jpeg_buf, jpeg_len, rgb_buf, 320, 240);

// 2. 使用RGB565检测（高精度）
SPOT_Detect_Process_Decoded(rgb_buf, 320, 240);
// 或者直接调用
SPOT_Detect_RGB565_Centroid(rgb_buf, 320, 240);
```

### 手动设置（测试用）

```c
// 测试追踪功能（手动设置光斑坐标）
SPOT_Detect_SetCenter(100, 80);  // 设置光斑在(100, 80)
SPOT_Detect_SetFound(1);          // 标记检测到光斑
ServoTrack_SetMode(SERVO_MODE_AUTO_TRACK);  // 切换到自动模式
// 舵机应该追踪到对应角度
```

### 检测算法说明

**质心法（推荐）**：
- 全局扫描找最亮点
- 在最亮点周围30x30窗口内计算质心
- 使用亮度平方加权，抗光晕干扰
- 适合单光源场景

**滑动窗口法**：
- 4x降采样找候选点
- 50x50窗口滑动搜索
- 适合多光源场景

## ?? 注意事项

1. **舵机供电**: 确保 PCA9685 供电充足（外部5V供电）
2. **角度限制**: 舵机角度自动限制在 0~180度
3. **死区设置**: 死区太大会导致中心区域不响应
4. **平滑系数**: 系数太小会导致响应过慢
5. **光斑检测**: 需要 SPOT_DETECT 模块正常工作
6. **调试输出**: 默认每30帧输出一次追踪信息

## ? 性能指标

- **追踪频率**: 15fps (取决于摄像头帧率)
- **响应延迟**: 66ms (1帧) + 平滑延迟
- **角度精度**: ±1度 (受舵机精度限制)
- **死区精度**: ±10 像素 (可调)

## ? 常见问题

### Q1: 舵机不动？
- 检查模式是否为 AUTO_TRACK
- 检查是否检测到光斑（`SPOT_Detect_IsFound()`）
- 检查光斑是否在死区内

### Q2: 舵机抖动？
- 增大平滑系数 (如 0.2)
- 增大死区半径 (如 15)
- 检查光斑检测稳定性

### Q3: 响应太慢？
- 减小平滑系数 (如 0.5)
- 减小死区半径
- 检查视频帧率

### Q4: 角度不准确？
- 检查舵机中位偏移设置
- 验证坐标映射公式
- 校准摄像头安装角度

## ? 版本历史

- **v1.0** (2025-01-xx)
  - 初始版本
  - 支持手动/自动模式
  - 平滑追踪算法
  - 中心死区功能
  - STM32F407适配

