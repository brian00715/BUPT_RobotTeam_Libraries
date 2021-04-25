/**
 * @file motor_driver.c
 * @author simon
 * @brief 电机驱动相关
 * @version 0.1
 * @date 2021-04-25
 * @note 
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "motor_driver.h"
#include <math.h>
#include "cmd.h"

#define VESC_BASE_ID 88
VESCMotor_t vesc = {0};
CANSendFlag_t CANSendFlag = {0};

/**
 * @brief 初始化本杰明电调
 **/
void VESC_Init(VESCMotor_t *vesc, uint8_t id, char mode)
{
  vesc->id = id;
  vesc->mode = mode;
  vesc->target_current = 0;
  vesc->target_duty = 0;
  vesc->target_rpm = 0;
}

/**
 * @brief 本杰明电调的执行函数，从cmd获取数据
 *        根据vesc结构体的mode执行相应功能
 *        mode0 占空比
 *        mode1 电流环
 *        mode2 速度环
 *        mode3 位置环（通常不用）
 **/
void VESC_Exe()
{
  if (CANSendFlag.vesc == 0)
  {
    return;
  }
  if (vesc.mode == 0)
  {
    comm_can_set_duty(vesc.id, vesc.target_duty);
  }
  if (vesc.mode == 1)
  {
    comm_can_set_current(vesc.id, vesc.target_current);
  }
  if (vesc.mode == 2)
  {
    comm_can_set_rpm(vesc.id, vesc.target_rpm);
  }
  if (vesc.mode == 3)
    comm_can_set_pos(vesc.id, vesc.target_position); //位置环
  CANSendFlag.vesc = 0;
}

float MoterDriver_M2006_Current = 0;
/**
 * @brief m2006大疆电机的执行函数,从cmd获取目标电流
 **/
void m2006_Exe()
{
  if (CANSendFlag.m2006 == 0) // 控制5ms发一次
    return;

  CANSendFlag.m2006 = 0;
}

int VESC_StatusBag_Flag = 0;
int VESC_SwitchPrintInfo_Flag = 0;
/**
 * @brief 本杰明电调反馈状态包解析函数
 **/
void VESC_RxHandler(can_msg *data)
{
  int32_t index = 0;
  vesc.now_position = buffer_get_int16(data->ui8, &index) / 50;
  vesc.now_rpm = buffer_get_int32(data->ui8, &index);
}

void VESC_PrintInfo(VESCMotor_t *vesc)
{
  if (VESC_SwitchPrintInfo_Flag)
  {
    uprintf("--vesc status>>");
    uprintf(" current_angle:%d current_rpm:%d mode:%d ", vesc->now_position, vesc->now_rpm, vesc->mode);
    switch (vesc->mode)
    {
    case 0:
      uprintf("duty:%.2f\r\n", vesc->target_duty);
      break;
    case 1:
      uprintf("current:%.2f\r\n", vesc->target_current);
      break;
    case 2:
      uprintf("rpm:%.2f\r\n", vesc->target_rpm);
      break;
    case 3:
      uprintf("pos:%.2f\r\n", vesc->target_position);
      break;
    default:
      uprintf("##Vesc Mode Error!##\r\n");
      break;
    }
  }
}

