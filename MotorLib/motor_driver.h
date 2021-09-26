#ifndef __MOTOR_DRIVER_H
#define __MOTOR_DRIVER_H

/* 对外接口-----------------------------------------*/
// 使用什么驱动器
// #define USE_MTR_DRIVER_ODRIVE
#define USE_MTR_DRIVER_VESC
#define USE_MTR_DRIVER_DJI_BOARD // 一次控制四个电机的大疆驱动板
// #define USE_MTR_DRIVER_RM_CXXX // C610/620电调

#include "can_utils.h"
#include "main.h"
#include "vesc_can.h"
#include "odrive_can.h"

extern int VESC_StatusBag_Flag;
extern int VESC_SwitchPrintInfo_Flag;
extern int VESC_SwitchStopByAngle_Flag;

typedef enum MTR_CTRL_MODE
{
    MTR_CTRL_CURRENT = 0,
    MTR_CTRL_RPM,
    MTR_CTRL_POSITION,
    MTR_CTRL_DUTY,
    MTR_CTRL_VOLTAGE // 电压模式和占空比模式差不多，取决于驱动器特性
} MTR_CTRL_MODE;

typedef struct CANSendFlag_t
{
    unsigned m2006 : 1;
    unsigned vesc : 1;
    unsigned odrive : 1;
} CANSendFlag_t; // 驱动器CAN消息发送标志位

typedef struct MotorDriver_t
{
    uint8_t can_id;        // can消息id
    uint8_t mode;          // 模式（速度环、电流环等）
    float target_duty;     // 占空比
    float target_current;  // 电流
    float target_rpm;      // 转速
    float target_position; // 位置
    float now_duty;
    float now_current;
    float now_rpm;
    float now_position;
} MotorDriver_t; // 电机驱动器抽象结构体

void VESC_Exe();
void RM2006_Exe();
void CAN_Callback_VESC(MotorDriver_t *vesc, CAN_Message_u *data);
void MotorDriver_Init(MotorDriver_t *motor_driver, uint8_t can_id, MTR_CTRL_MODE mode);

#endif //_MOTOR_DRIVER_H