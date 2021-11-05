
/**
 * @file can_func.c
 * @author BUPT Robot Team 19th
 * @brief 对外接口，添加CAN中断回调函数
 * @version 0.1
 * @date 
 * 
 * @copyright Copyright (c) 2021
 * 
 */

/* 在此处添加包含CAN回调函数的头文件----------------------------------------------------*/
// #include "handle.h"
// #include "dji_board_can_v1.h"
// #include "base_chassis.h"

#include "can_func.h"
#include "cmd.h"
#include "can_utils.h"
#include "utils.h"

#ifdef SLIB_USE_CAN // 使能CAN

uint32_t CAN_AcceptID_Std[8] = {324, 325, 0x281, 0x282, 204, 205, 206}; // 希望接收的ID，与回调函数ID对应

/**
 * @brief 注册CAN回调函数
 * 
 */
void CAN_FuncInit()
{
    // CAN_CallbackAdd(324, CAN_Callback_Handle_Rocker);
    // CAN_CallbackAdd(325, CAN_Callback_Handle_Button);
    // CAN_CallbackAdd(0x281, CAN_Callback_DJIBoard_ReadInfo);
    // CAN_CallbackAdd(0x282, CAN_Callback_DJIBoard_ReadAllPosInfo);
    // CAN_CallbackAdd(204, CAN_Callback_Locator_ReadPos_X);
    // CAN_CallbackAdd(205, CAN_Callback_Locator_ReadPos_Y);
    // CAN_CallbackAdd(206, CAN_Callback_Locator_ReadPos_Yaw);
}

#endif // SLIB_USE_CAN