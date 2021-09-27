/*******************************************************************************
 * Copyright:		BUPT
 * File Name:		cmd.h
 * Description:		ָ��ʵ��
 * Author:			ZeroVoid
 * Version:			0.1
 * Data:			2019/09/23 Mon 13:46
 *******************************************************************************/

#ifndef __CMD_H
#define __CMD_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "simplelib_cfg.h"

#ifdef SLIB_USE_CMD
#include "main.h"
#include "usart.h"

#ifdef SLIB_USE_NRF_COMM
#include "nrf_comm.h"
#endif // SLIB_USE_NRF_COMM

#if defined(STM32F407xx) || defined(STM32F405xx)
#include "stm32f4xx_hal.h"
#endif // STM32F407xx

#define MAX_CMD_ARG_LENGTH 16
#define MAX_CMD_INFO_LENGTH 64
#define MAX_CMD_LINE_LENGTH 128
#define MAX_ARGV 12 // 最大命令参数数量
#define PRINT_BUFFER_SIZE 128
#define DMA_BUFFER_SIZE 128

#ifdef SLIB_USE_NRF_COMM
#define DMA_BUFFER_SIZE (32 - NRF_PCK_HEADER_SIZE)
#else

#endif // SLIB_USE_NRF_COMM

    /* cmd_huart允许simplelib其他组件访问, CMD_UART宏用于兼容旧版本 */
    extern UART_HandleTypeDef *slib_cmd_huart;
    #define CMD_UART    (*slib_cmd_huart)

    extern uint8_t UART_DMA_RxOK_Flag;
    extern char cmd_line[MAX_CMD_LINE_LENGTH + 1];
    extern char *cmd_argv[MAX_ARGV];

    struct cmd_info
    {
        void (*cmd_func)(int argc, char *argv[]); // 函数指针
        char *cmd_usage;
    };

    void USART_DMA_Init();
    void USART_DMA_Exe();
    void HAL_UART_IDLECallback(UART_HandleTypeDef *huart);

    int CMD_Parse(char *cmd_line, int *argc, char *argv[]);
    int CMD_Exec(int argc, char *argv[]);
    void CMD_HelpFunc(int argc, char *argv[]);
    void CMD_Init(void);
    void CMD_Add(char *cmd_name, char *cmd_usage, void (*cmd_func)(int argc, char *argv[]));

    void send_wave(float arg1, float arg2, float arg3, float arg4);
    void uprintf(char *fmt, ...);
    void uprintf_to(UART_HandleTypeDef *huart, char *fmt, ...);

#endif // SLIB_USE_CMD

#ifdef __cplusplus
}
#endif
#endif /*__CMD_H */