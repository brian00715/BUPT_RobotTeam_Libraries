/**
 * @file sys_func.c
 * @author simon
 * @brief STM32自定义系统功能函数
 * @version 0.1
 * @date 2021-05-31
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "sys_func.h"

/**
 * @brief for TIMs using APB1(42MHz)
 * 
 * @param timx 
 * @param freq 
 */
void TIM_SetFreq1(TIM_HandleTypeDef *timx, int16_t freq)
{
    HAL_TIM_Base_DeInit(timx);
    timx->Init.AutoReloadPreload = (uint32_t)(42000000 / (timx->Instance->PSC + 1) / freq);
    HAL_TIM_Base_Init(timx);
}
