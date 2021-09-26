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

asm(".global _printf_float");

/**
 * @brief	初始化配置
 * @param	cmd_usart   指令通信usart句柄
 * @param   hcan        CAN通信句柄
 * @return	None
 */
void simplelib_init(UART_HandleTypeDef *cmd_usart, CAN_HandleTypeDef *hcan)
{
#ifdef SL_USART_DMA
    if (cmd_usart != NULL)
    {
        usart_DMA_init(cmd_usart);
    }
#endif // SL_USART_DMA

#ifdef SL_CMD
    cmd_func_init();
    uprintf("==simplelib init done==\r\n");
#endif // SL_CMD

#ifdef SL_CAN
    if (hcan != NULL)
    {
        CAN_Init(hcan);
        CAN_FuncInit();
    }
#endif // SL_CAN
}

void simplelib_run(void)
{
#ifdef SL_CMD
    if (DMA_RxOK_Flag)
    {
        USART_DMA_Exe();
        DMA_RxOK_Flag = 0;
    }
#endif // SL_CMD
#ifdef SL_CAN
    if (CAN_ExeCallback_Flag)
    {
        CAN_CallbackExe();
        CAN_ExeCallback_Flag = 0;
    }
#endif // SL_CAN
    if (send_wave_flag)
    {
        //HAL_Delay(10);
    }
}
