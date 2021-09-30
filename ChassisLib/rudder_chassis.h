#ifndef RUDDER_CHASSIS_H_
#define RUDDER_CHASSIS_H_

#include "base_chassis.h"
#include "chassis_common_config.h"
#ifdef USE_CHASSIS_RUDDER

#include "steer_wheel.h"
#include "chassis_common_config.h"
#include "handle.h"
#include "main.h"
#include "vec.h"
#include "point.h"

#ifdef __cplusplus
extern "C"
{
#endif
    //=================================宏和ENUM=================================

    //=================================结构体=================================

    typedef struct RudderChassis_s // 底盘抽象结构体
    {
        BaseChassis_s *base;
        SW_DriveMotor_t DriveMotors;
        SW_SteerMotor_t SteerMotors;
        uint8_t send_ctrl_msg_flag;
    } RudderChassis_s;

    //=================================全局变量=================================

    extern RudderChassis_s RudderChassis; // 底盘全局结构体

    //=================================函数声明=================================

    void RudderChassis_Move(float vel, float dir, float omega);
    void RudderChassis_Init();
    void RudderChassis_SingleCtrl(int index, float speed, float dir);
    void RudderChassis_CtrlYaw(float yaw, uint32_t max_omega);
    void RudderChassis_Exe();
    void RudderChassis_TurnMinorArc(float now_dir, float *target_dir, float *target_speed);
    void RudderChassis_SuddenStop(float acc);
    void RudderChassis_YawTuning(float target_yaw);
    void RudderChassis_Rotate(float target_yaw);

#ifdef __cplusplus
}
#endif
#endif

#endif