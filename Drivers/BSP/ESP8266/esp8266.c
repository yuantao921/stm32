/**
 ****************************************************************************************************
 * @file        esp8266.c
 * @author      用户
 * @version     V1.0
 * @date        2024
 * @brief       ESP8266 WiFi模块驱动
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 探索者 F407开发板
 * 功能说明:通过ESP8266实现JPEG图像的无线传输
 *
 * 使用说明:
 * 1. ESP8266连接到STM32的串口2(PA2-TX, PA3-RX)
 * 2. ESP8266需要单独供电(3.3V, 建议500mA以上)
 * 3. 默认配置为TCP服务器模式，端口8080
 * 4. 电脑端使用网络调试助手或串口工具连接ESP8266的IP地址和端口
 *
 ****************************************************************************************************
 */

#include "./BSP/ESP8266/esp8266.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/USART2/usart2.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef g_uart2_handle;  /* 外部声明串口2句柄 */

static char g_esp8266_rx_buf[512];     /* ESP8266接收缓冲区 */
static uint16_t g_esp8266_rx_len = 0; /* 接收数据长度 */

/**
 * @brief       ESP8266串口初始化
 * @param       baudrate: 波特率
 * @retval      无
 * @note        重新配置串口2为收发模式，以支持ESP8266的AT指令响应接收
 */
void esp8266_uart_init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio_init_struct;
    
    /* 重新配置串口2为收发模式 */
    USART2_UX_CLK_ENABLE();
    USART2_TX_GPIO_CLK_ENABLE();
    USART2_RX_GPIO_CLK_ENABLE();
    
    gpio_init_struct.Pin = USART2_TX_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate = USART2_TX_GPIO_AF;
    HAL_GPIO_Init(USART2_TX_GPIO_PORT, &gpio_init_struct);
    
    gpio_init_struct.Pin = USART2_RX_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Alternate = USART2_RX_GPIO_AF;
    HAL_GPIO_Init(USART2_RX_GPIO_PORT, &gpio_init_struct);
    
    g_uart2_handle.Instance = USART2_UX;
    g_uart2_handle.Init.BaudRate = baudrate;
    g_uart2_handle.Init.WordLength = UART_WORDLENGTH_8B;
    g_uart2_handle.Init.StopBits = UART_STOPBITS_1;
    g_uart2_handle.Init.Parity = UART_PARITY_NONE;
    g_uart2_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_uart2_handle.Init.Mode = UART_MODE_TX_RX;  /* 收发模式 */
    HAL_UART_Init(&g_uart2_handle);
}

/**
 * @brief       发送AT指令并等待响应
 * @param       cmd: AT指令字符串
 * @param       ack: 期望的响应字符串，NULL表示不检查响应
 * @param       waittime: 等待时间(ms)
 * @retval      0: 成功; 1: 失败或超时
 */
uint8_t esp8266_send_cmd(char *cmd, char *ack, uint16_t waittime)
{
    uint8_t res = 0;
    uint32_t start_time = 0;
    char *p;
    
    if (cmd == NULL) return 1;
    
    /* 清空接收缓冲区 */
    g_esp8266_rx_len = 0;
    memset(g_esp8266_rx_buf, 0, sizeof(g_esp8266_rx_buf));
    
    /* 发送AT指令 */
    printf("Send: %s\r\n", cmd);
    while (*cmd)
    {
        while ((USART2->SR & 0X40) == 0);  /* 等待发送完成 */
        USART2->DR = *cmd++;
    }
    
    /* 等待响应 */
    if (ack == NULL) return 0;  /* 不需要检查响应 */
    
    start_time = HAL_GetTick();
    while ((HAL_GetTick() - start_time) < waittime)
    {
        /* 接收数据（使用轮询方式） */
        if (__HAL_UART_GET_FLAG(&g_uart2_handle, UART_FLAG_RXNE) != RESET)  /* 接收数据就绪 */
        {
            if (g_esp8266_rx_len < sizeof(g_esp8266_rx_buf) - 1)
            {
                g_esp8266_rx_buf[g_esp8266_rx_len++] = (uint8_t)(g_uart2_handle.Instance->DR & 0xFF);
                g_esp8266_rx_buf[g_esp8266_rx_len] = '\0';
            }
        }
        
        /* 检查是否包含期望的响应 */
        p = strstr((char *)g_esp8266_rx_buf, ack);
        if (p != NULL)
        {
            printf("Recv: %s\r\n", g_esp8266_rx_buf);
            res = 0;
            break;
        }
        
        delay_ms(10);
    }
    
    if (res != 0)
    {
        printf("Timeout! Recv: %s\r\n", g_esp8266_rx_buf);
    }
    
    return res;
}

/**
 * @brief       恢复ESP8266出厂设置
 * @param       无
 * @retval      0: 成功; 1: 失败
 */
uint8_t esp8266_restore(void)
{
    uint8_t res = 0;
    
    printf("ESP8266 Restore Factory Settings...\r\n");
    res = esp8266_send_cmd("AT+RESTORE\r\n", "OK", 5000);
    delay_ms(2000);  /* 恢复出厂设置需要更长时间 */
    
    return res;
}

/**
 * @brief       复位ESP8266
 * @param       无
 * @retval      0: 成功; 1: 失败
 */
uint8_t esp8266_reset(void)
{
    uint8_t res = 0;
    
    printf("ESP8266 Reset...\r\n");
    res = esp8266_send_cmd("AT+RST\r\n", "ready", 5000);
    delay_ms(1000);
    
    return res;
}

/**
 * @brief       设置ESP8266工作模式
 * @param       mode: 工作模式 (1:Station, 2:AP, 3:AP+Station)
 * @retval      0: 成功; 1: 失败
 */
uint8_t esp8266_set_wifi_mode(uint8_t mode)
{
    char cmd[20];
    
    sprintf(cmd, "AT+CWMODE=%d\r\n", mode);
    return esp8266_send_cmd(cmd, "OK", ESP8266_CMD_TIMEOUT);
}

/**
 * @brief       连接WiFi热点
 * @param       ssid: WiFi名称
 * @param       pwd: WiFi密码
 * @retval      0: 成功; 1: 失败
 */
uint8_t esp8266_connect_ap(char *ssid, char *pwd)
{
    char cmd[100];
    
    if (ssid == NULL) return 1;
    
    if (pwd == NULL || strlen(pwd) == 0)
    {
        sprintf(cmd, "AT+CWJAP=\"%s\"\r\n", ssid);
    }
    else
    {
        sprintf(cmd, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, pwd);
    }
    
    return esp8266_send_cmd(cmd, "OK", 10000);  /* 连接WiFi可能需要更长时间 */
}

/**
 * @brief       设置TCP服务器模式
 * @param       port: 服务器端口号
 * @retval      0: 成功; 1: 失败
 */
uint8_t esp8266_set_tcp_server(uint16_t port)
{
    char cmd[30];
    
    /* 关闭多连接 */
    esp8266_send_cmd("AT+CIPMUX=1\r\n", "OK", ESP8266_CMD_TIMEOUT);
    delay_ms(100);
    
    /* 设置TCP服务器 */
    sprintf(cmd, "AT+CIPSERVER=1,%d\r\n", port);
    return esp8266_send_cmd(cmd, "OK", ESP8266_CMD_TIMEOUT);
}

/**
 * @brief       设置透传模式
 * @param       mode: 1-开启透传, 0-关闭透传
 * @retval      0: 成功; 1: 失败
 */
uint8_t esp8266_set_passthrough(uint8_t mode)
{
    char cmd[20];
    
    if (mode)
    {
        sprintf(cmd, "AT+CIPMODE=1\r\n");  /* 开启透传模式 */
    }
    else
    {
        sprintf(cmd, "AT+CIPMODE=0\r\n");  /* 关闭透传模式 */
    }
    
    return esp8266_send_cmd(cmd, "OK", ESP8266_CMD_TIMEOUT);
}

/**
 * @brief       连接TCP服务器
 * @param       ip: 服务器IP地址
 * @param       port: 服务器端口号
 * @retval      0: 成功; 1: 失败
 */
uint8_t esp8266_connect_tcp_server(char *ip, uint16_t port)
{
    char cmd[50];
    
    if (ip == NULL) return 1;
    
    sprintf(cmd, "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", ip, port);
    return esp8266_send_cmd(cmd, "OK", 10000);  /* 连接可能需要更长时间 */
}

/**
 * @brief       开启透传（发送AT+CIPSEND）
 * @param       无
 * @retval      0: 成功; 1: 失败
 */
uint8_t esp8266_start_passthrough(void)
{
    printf("Start Passthrough Mode...\r\n");
    return esp8266_send_cmd("AT+CIPSEND\r\n", ">", ESP8266_CMD_TIMEOUT);
}

/**
 * @brief       获取ESP8266的IP地址
 * @param       ip: IP地址存储缓冲区(至少16字节)
 * @retval      0: 成功; 1: 失败
 */
uint8_t esp8266_get_ip(char *ip)
{
    char *p;
    
    if (ip == NULL) return 1;
    
    if (esp8266_send_cmd("AT+CIFSR\r\n", NULL, ESP8266_CMD_TIMEOUT) == 0)
    {
        delay_ms(500);
        /* 简单解析IP地址，实际应该更完善 */
        p = strstr((char *)g_esp8266_rx_buf, "+CIFSR:STAIP,\"");
        if (p != NULL)
        {
            p += 13;  /* 跳过"+CIFSR:STAIP,\"" */
            sscanf(p, "%[^\"]", ip);
            return 0;
        }
    }
    
    return 1;
}

/**
 * @brief       ESP8266初始化（透传模式）
 * @param       无
 * @retval      0: 成功; 1: 失败
 * 
 * @note       按照透传模式初始化:
 *             1. AT+RESTORE - 恢复出厂设置
 *             2. AT+CWMODE=1 - 设置Station模式
 *             3. AT+RST - 复位
 *             4. AT+CWJAP="SSID","PWD" - 连接WiFi
 *             5. AT+CIPMODE=1 - 设置透传模式
 *             6. AT+CIPSTART="TCP","IP",PORT - 连接TCP服务器
 *             7. AT+CIPSEND - 开启透传
 *             
 *             需要修改的参数:
 *             - WiFi名称和密码（在esp8266_connect_ap调用中）
 *             - TCP服务器IP和端口（在esp8266_connect_tcp_server调用中）
 */
uint8_t esp8266_init(void)
{
    uint8_t res = 0;
    
    printf("ESP8266 Init (Passthrough Mode)...\r\n");
    
    /* 初始化串口 */
    esp8266_uart_init(ESP8266_BAUDRATE);
    delay_ms(1000);
    
    /* 1. 恢复出厂设置 */
    printf("Step 1: Restore factory settings...\r\n");
    esp8266_restore();
    delay_ms(2000);
    
    /* 2. 设置WiFi模式为Station */
    printf("Step 2: Set WiFi mode to Station...\r\n");
    res = esp8266_set_wifi_mode(ESP8266_MODE_STATION);
    if (res) 
    {
        printf("Set WiFi mode failed!\r\n");
        return 1;
    }
    delay_ms(500);
    
    /* 3. 复位ESP8266 */
    printf("Step 3: Reset ESP8266...\r\n");
    res = esp8266_reset();
    if (res) 
    {
        printf("Reset failed!\r\n");
        return 1;
    }
    delay_ms(2000);
    
    /* 4. 连接WiFi (需要根据实际情况修改) */
    printf("Step 4: Connect to WiFi...\r\n");
    /* 注意: 这里需要修改为你的WiFi名称和密码 */
    res = esp8266_connect_ap("EVEB", "123456789");
    if (res)
    {
        printf("WiFi Connect Failed! Please check SSID and Password!\r\n");
        printf("Current SSID: EVEB, Password: 123456789\r\n");
        printf("Please modify in esp8266_init() function!\r\n");
        return 1;
    }
    delay_ms(2000);
    
    /* 获取IP地址 */
    char ip[16] = {0};
    if (esp8266_get_ip(ip) == 0)
    {
        printf("ESP8266 IP: %s\r\n", ip);
    }
    
    /* 5. 设置透传模式 */
    printf("Step 5: Set passthrough mode...\r\n");
    res = esp8266_set_passthrough(ESP8266_PASSTHROUGH_ON);
    if (res) 
    {
        printf("Set passthrough mode failed!\r\n");
        return 1;
    }
    delay_ms(500);
    
    /* 6. 连接TCP服务器 (需要根据实际情况修改) */
    printf("Step 6: Connect to TCP server...\r\n");
    /* 注意: 这里需要修改为电脑的IP地址和端口 */
    /* 默认: 192.168.0.8:8088 */
    res = esp8266_connect_tcp_server("192.168.0.8", 8088);
    if (res)
    {
        printf("Connect TCP server failed! Please check:\r\n");
        printf("1. PC TCP server is running on 192.168.0.8:8088\r\n");
        printf("2. PC and ESP8266 are in same WiFi network\r\n");
        printf("3. Firewall allows connection\r\n");
        return 1;
    }
    delay_ms(1000);
    
    /* 7. 开启透传 */
    printf("Step 7: Start passthrough...\r\n");
    res = esp8266_start_passthrough();
    if (res) 
    {
        printf("Start passthrough failed!\r\n");
        return 1;
    }
    delay_ms(500);
    
    printf("ESP8266 Init OK! Passthrough mode enabled\r\n");
    printf("Data will be sent directly to TCP server\r\n");
    
    return 0;
}

