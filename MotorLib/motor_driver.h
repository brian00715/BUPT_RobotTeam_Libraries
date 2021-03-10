#ifndef __MOTOR_DRIVER_H
#define __MOTOR_DRIVER_H

/* 对外接口-----------------------------------------*/
// 使用什么驱动器
// #define USE_MTR_DRIVER_ODRIVE       // ODrive驱动器
#define USE_MTR_DRIVER_VESC         // VESC驱动器
#define USE_MTR_DRIVER_DJI_BOARD_V1 // 一次控制四个电机的大疆驱动板
// #define USE_MTR_DRIVER_DJI_BOARD_V2 // 使用V2版时务必配合OSLib一起使用
#define USE_MTR_DRIVER_DRIVER_PRO   // BUPT自研黑色驱动卡
// #define USE_MTR_DRIVER_RM_CXXX      // C610/620电调

#include "can_utils.h"

typedef enum MTR_CTRL_MODE
{
    MTR_CTRL_CURRENT = 0,
    MTR_CTRL_RPM,
    MTR_CTRL_POSITION,
    MTR_CTRL_DUTY,
    MTR_CTRL_VOLTAGE // 电压模式和占空比模式差不多，取决于驱动器特性
} MTR_CTRL_MODE;

typedef struct CANSendFlag_s
{
    unsigned m2006 : 1;
    unsigned vesc : 1;
    unsigned odrive : 1;
} CANSendFlag_s; // 驱动器CAN消息发送标志位

typedef struct MotorDriver_s
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
} MotorDriver_s; // 电机驱动器抽象结构体

void VESC_Exe();
void MotorDriver_Init(MotorDriver_s *motor_driver, uint8_t can_id, MTR_CTRL_MODE mode);

#endif // __MOTOR_DRIVER_H