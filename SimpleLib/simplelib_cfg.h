/*******************************************************************************
 * Copyright:		BUPT
 * File Name:		simplelib_config.h
 * Description:		SimpleLib: 兼容STM32CubeMX生成项目的组件Lib
 * Version:			0.1.1
 * Author:			ZeroVoid, Team 20th
 * Data:			2021-09-27 
 * Encoding:        UTF-8
 *******************************************************************************/
#ifndef __SIMPLELIB_CONFIG_H
#define __SIMPLELIB_CONFIG_H
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * SimpleLib Configuration
 * @note	SL Simplelib or Stm32Lib
 * @note    VSCode comment shortcut: ctrl + /
 *******************************************************************************/
#define SLIB_USE_CAN   // 使能CAN通信模块
#define SLIB_USE_CMD   // 使能串口通信、串口命令行模块
#define SLIB_USE_UTILS // 通用工具函封装
#define SLIB_USE_UART_DMA
// #define SLIB_USE_TOOLBOXSCOPE // 启用串口示波器
// #define SLIB_USE_FLASH    // Flash 模块
// #define SLIB_USE_NRF      // NRF通信模块
// #define SLIB_USE_NRF_COMM // NRF 通信通信协议
// #define SLIB_USE_IMI_SPI  // 软件SPI封装
// #define SLIB_USE_AS5047P  // AS5047P
// #define SLIB_USE_DEBUG    // Simplelib debug macro
// #define SLIB_USE_NRF_DEBUG

/* Flash Configuration -----------------------------------------------------*/
#ifdef SLIB_USE_FLASH
#define FLASH_SIZE 25 // Flash float数组大小
#endif                // SLIB_USE_FLASH

/* NRF Configuration -----------------------------------------------------*/
#ifdef SLIB_USE_NRF
// #define SL_NRF_DMA                      // 使用DMA通信
#include "gpio.h"
#include "spi.h"

#ifdef STM32F072xB
#define NRF_ADDR_COF 3 // 接收地址变化参数,方便测试
#define NRF_SPI_Handle hspi1
#define NRF_SPI_CSN_PIN NRF_CS_Pin
#define NRF_SPI_CSN_GPIO_PORT NRF_CS_GPIO_Port
#define NRF_SPI_IRQ_PIN NRF_IRQ_Pin
#define NRF_SPI_IRQ_GPIO_PORT NRF_IRQ_GPIO_Port
#define NRF_SPI_CE_PIN NRF_CE_Pin
#define NRF_SPI_CE_GPIO_PORT NRF_CE_GPIO_Port
#endif // STM32F072xB

#ifdef STM32F407xx
#define NRF_ADDR_COF 1 // 接收地址变化参数,方便测试
#define NRF_SPI_Handle hspi3
#define NRF_SPI_CSN_PIN GPIO_PIN_15
#define NRF_SPI_CSN_GPIO_PORT GPIOA
#define NRF_SPI_IRQ_PIN GPIO_PIN_8
#define NRF_SPI_IRQ_GPIO_PORT GPIOB
#define NRF_SPI_CE_PIN GPIO_PIN_9
#define NRF_SPI_CE_GPIO_PORT GPIOB
#endif // STM32F407xx

#endif // SLIB_USE_NRF

#ifdef __cplusplus
}
#endif

#endif /* __SIMPLELIB_CONFIG_H */