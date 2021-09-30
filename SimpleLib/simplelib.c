/*******************************************************************************
 * Copyright:		BUPT
 * File Name:		simplelib.c
 * Author:			ZeroVoid
 * Description:		None
 * Bug:				None
 * Version:			0.2.1
 * Data:			2019/09/19 Thu 19:35
 * Todo:			None
 *******************************************************************************/

#include "simplelib.h"

// asm(".global _printf_float");

/**
 * @brief	初始化配置
 * @param	cmd_usart   指令通信usart句柄
 * @param   hcan        CAN通信句柄
 * @return	None
 */
void SimpleLib_Init(UART_HandleTypeDef *cmd_usart, CAN_HandleTypeDef *hcan)
{
#ifdef SLIB_USE_UART_DMA
    if (cmd_usart != NULL)
    {
        USART_DMA_Init(cmd_usart);
    }
#endif // SLIB_USE_UART_DMA

#ifdef SLIB_USE_CMD
    CMD_FuncInit();
    uprintf("==simplelib init done==\r\n");
#endif // SLIB_USE_CMD

#ifdef SLIB_USE_CAN
    if (hcan != NULL)
    {
        CAN_Init(hcan);
        CAN_FuncInit();
    }
#endif // SLIB_USE_CAN
}

void SimpleLib_Run(void)
{
#ifdef SLIB_USE_CMD
    if (UART_DMA_RxOK_Flag)
    {
        USART_DMA_Exe();
        UART_DMA_RxOK_Flag = 0;
    }
#endif // SLIB_USE_CMD
#ifdef SLIB_USE_CAN
    if (CAN_ExeCallback_Flag)
    {
        CAN_CallbackExe();
        CAN_ExeCallback_Flag = 0;
    }
#endif // SLIB_USE_CAN
}
