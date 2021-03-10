#ifndef OMNI_CHASSIS_H_
#define OMNI_CHASSIS_H_

#include "motor_driver.h"
#include "base_chassis.h"

#define OMNI_DRIVE_MOTOR_BASE_CANID 50 // 驱动轮CAN ID基址

typedef struct OmniChassis_t
{
    BaseChassis_s *base; // 该结构体一直存在，子类只需关联其指针
    // VESCDriver_t drive_motors[4];
    MotorDriver_s drive_motors[4];
    char motor_can_flag; // 在定时中断中置1，控制can消息发送频率
} OmniChassis_t;

void Omnihassis_Move(float target_speed, float target_dir, float target_omega);
void OmniChassis_Exe();
void OmniChassis_Init();

extern char MotorCANSendFlag; // 需要在定时中断中置1，控制发送频率
extern OmniChassis_t OmniChassis;

#endif