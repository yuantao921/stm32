/**
 ****************************************************************************************************
 * @file        esp8266.h
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
 ****************************************************************************************************
 */

#ifndef _ESP8266_H
#define _ESP8266_H

#include "./SYSTEM/sys/sys.h"
#include "./BSP/USART2/usart2.h"

/******************************************************************************************/
/* ESP8266配置参数 */
#define ESP8266_BAUDRATE           115200      /* ESP8266通信波特率，建议115200或230400 */
#define ESP8266_RX_TIMEOUT         3000        /* 接收超时时间(ms) */
#define ESP8266_CMD_TIMEOUT        5000        /* AT指令超时时间(ms) */

/* ESP8266工作模式 */
#define ESP8266_MODE_STATION       1           /*  Station模式 */
#define ESP8266_MODE_AP            2           /*  AP模式 */
#define ESP8266_MODE_AP_STATION    3           /*  AP+Station模式 */

/* ESP8266透传模式 */
#define ESP8266_PASSTHROUGH_ON     1           /*  开启透传 */
#define ESP8266_PASSTHROUGH_OFF    0           /*  关闭透传 */

/******************************************************************************************/
/* 函数声明 */

uint8_t esp8266_send_cmd(char *cmd, char *ack, uint16_t waittime);
uint8_t esp8266_init(void);
uint8_t esp8266_reset(void);
uint8_t esp8266_restore(void);
uint8_t esp8266_set_wifi_mode(uint8_t mode);
uint8_t esp8266_connect_ap(char *ssid, char *pwd);
uint8_t esp8266_connect_tcp_server(char *ip, uint16_t port);
uint8_t esp8266_set_tcp_server(uint16_t port);
uint8_t esp8266_set_passthrough(uint8_t mode);
uint8_t esp8266_start_passthrough(void);
uint8_t esp8266_get_ip(char *ip);
void esp8266_uart_init(uint32_t baudrate);

#endif

