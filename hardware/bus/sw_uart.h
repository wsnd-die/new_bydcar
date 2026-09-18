/**
 * @file    sw_uart.h
 * @brief   软件串口模块 — 使用 PB6(RX)/PB7(TX) 实现 bit-bang 半双工/全双工 UART
 * @note
 *   - 启用本模块前，请确保 OLED_USE_SW_I2C 在 oled.c 中已被注释，
 *     因为两者共用 PB6/PB7 引脚，不能同时使用。
 *   - 通过 SW_UART_ENABLE 宏控制：1 = 启用软件串口，0 = 禁用。
 *   - 波特率通过 SW_UART_BAUDRATE 配置。
 */

#ifndef SW_UART_H
#define SW_UART_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * 编译开关（1 = 启用，0 = 禁用）
 * ================================================================ */
#define SW_UART_ENABLE        1U

/* ================================================================
 * 引脚和波特率配置
 * ================================================================ */
/*
 * 波特率：115200（汇编延时，TX 关中断保护位时序）
 */
#define SW_UART_BAUDRATE       115200U
#define SW_UART_RX_PIN         GPIO_PIN_6    /* PB6 */
#define SW_UART_RX_PORT        GPIOB
#define SW_UART_TX_PIN         GPIO_PIN_7    /* PB7 */
#define SW_UART_TX_PORT        GPIOB

/* 接收环形缓冲区大小（字节） */
#define SW_UART_RX_BUF_SIZE    128U

#if SW_UART_ENABLE

/* ================================================================
 * 公开 API
 * ================================================================ */

/**
 * @brief  初始化软件串口（配置 GPIO、EXTI、DWT）
 * @note   会覆盖 PB6/PB7 的 GPIO 配置（原 I2C 模式）
 */
void SW_UART_Init(void);

/**
 * @brief  阻塞发送一个字节
 */
void SW_UART_SendByte(uint8_t data);

/**
 * @brief  阻塞发送字节数组
 */
void SW_UART_SendBytes(const uint8_t *data, uint16_t len);

/**
 * @brief  阻塞发送字符串（不含末尾 \0）
 */
void SW_UART_SendString(const char *str);

/**
 * @brief  简易 printf，最大 128 字节
 */
void SW_UART_Printf(const char *format, ...);

/**
 * @brief  接收缓冲区中可读的字节数
 */
uint16_t SW_UART_Available(void);

/**
 * @brief  从接收缓冲区读一个字节（无数据时返回 0）
 * @retval 接收到的字节
 */
uint8_t SW_UART_ReadByte(void);

/**
 * @brief  检查发送是否忙（TX 正在发送时返回 true）
 */
bool SW_UART_TxBusy(void);

/**
 * @brief  轮询 PB6 检测起始位（替代 EXTI，当 EXTI 中断不可用时）
 */
void SW_UART_PollStartBit(void);

#endif /* SW_UART_ENABLE */

#ifdef __cplusplus
}
#endif

#endif /* SW_UART_H */
