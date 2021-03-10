/**
 * @file driver_pro.c
 * @author simon
 * @brief 自研驱动卡can总线协议封装
 * @version 0.1
 * @date 2021-10-21
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "motor_driver.h"

#ifdef USE_MTR_DRIVER_DRIVER_PRO
#include "driver_pro_can.h"

/*************************************
 * @description: drivepro电机驱动（设置占空比）
 * @param {DrivePro_t} drivepro
 * @return {*}
*************************************/
void DrivePro_SetDuty(MotorDriver_s drivepro)
{
    CAN_Message_u can_tx_data;
    can_tx_data.in[0] = 0;
    can_tx_data.in[1] = (int)(drivepro.target_duty);
    CAN_SendStdMsg(drivepro.can_id, &can_tx_data);
}

/*************************************
 * @description: drivepro电机驱动（设置速度）
 * @param {DrivePro_t} drivepro
 * @return {*}
*************************************/
void DrivePro_SetRPM(MotorDriver_s drivepro)
{
    CAN_Message_u can_tx_data;
    can_tx_data.in[0] = 1;
    can_tx_data.in[1] = (int)(drivepro.target_rpm);
    CAN_SendStdMsg(drivepro.can_id, &can_tx_data);
}
#endif