/**
 * @file motor_driver.c
 * @author simon
 * @brief 电机驱动抽象函数
 * @version 0.1
 * @date 2021-04-25
 * @note 由于不能事先知道用几个电机，函数接口使用结构体指针，而不使用全局结构体
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "motor_driver.h"
#include <math.h>
#include "cmd.h"
#include "vesc_can.h"
#include "odrive_can.h"

CANSendFlag_s CANSendFlag = {0};

/**
 * @brief 电机驱动器基类初始化
 * 
 */
void MotorDriver_Init(MotorDriver_s *motor_driver, uint8_t can_id, MTR_CTRL_MODE mode)
{
  motor_driver->can_id = can_id;
  motor_driver->mode = mode;
  motor_driver->target_current = 0;
  motor_driver->target_duty = 0;
  motor_driver->target_rpm = 0;
  motor_driver->target_position = 0;
  motor_driver->now_current = 0;
  motor_driver->now_duty = 0;
  motor_driver->now_rpm = 0;
  motor_driver->now_position = 0;
}

/**
 * @brief 本杰明电调的执行函数，从cmd获取数据
 *        根据vesc结构体的mode执行相应功能
 *        mode0 占空比
 *        mode1 电流环
 *        mode2 速度环
 *        mode3 位置环（通常不用）
 **/
void VESC_Exe(MotorDriver_s *vesc)
{
  if (CANSendFlag.vesc == 0)
  {
    return;
  }
  if (vesc->mode == 0)
  {
    comm_can_set_duty(vesc->can_id, vesc->target_duty);
  }
  if (vesc->mode == 1)
  {
    comm_can_set_current(vesc->can_id, vesc->target_current);
  }
  if (vesc->mode == 2)
  {
    comm_can_set_rpm(vesc->can_id, vesc->target_rpm);
  }
  if (vesc->mode == 3)
    comm_can_set_pos(vesc->can_id, vesc->target_position); //位置环
  CANSendFlag.vesc = 0;
}

void MotorDriver_PrintInfo(MotorDriver_s *mtr)
{
  if (VESC_SwitchPrintInfo_Flag)
  {
    uprintf("--vesc status>>");
    uprintf(" current_angle:%d current_rpm:%d mode:%d ", mtr->now_position, mtr->now_rpm, mtr->mode);
    switch (mtr->mode)
    {
    case 0:
      uprintf("duty:%.2f\r\n", mtr->target_duty);
      break;
    case 1:
      uprintf("current:%.2f\r\n", mtr->target_current);
      break;
    case 2:
      uprintf("rpm:%.2f\r\n", mtr->target_rpm);
      break;
    case 3:
      uprintf("pos:%.2f\r\n", mtr->target_position);
      break;
    default:
      uprintf("##Vesc Mode Error!##\r\n");
      break;
    }
  }
}


