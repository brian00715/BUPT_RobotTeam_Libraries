#ifndef STEER_WHEEL_H_
#define STEER_WHEEL_H_

#include "base_chassis.h"
#include "chassis_common_config.h"
#include "motor_driver.h"
#ifdef USE_CHASSIS_RUDDER

/* 驱动轮使用什么驱动器----------------------------------------------*/
#define DRIVE_MTR_USE_VESC
// #define DRIVE_MTR_USE_ODRIVE

#ifdef __cplusplus
extern "C"
{
#endif
//=================================头文件包含=================================
#include "main.h"
#include "simplelib.h"
#include "can_utils.h"
#include "dji_board_can_v1.h"
    //=================================宏和枚举类=================================

    //=================================结构体=================================

    typedef struct SW_SteerMotor_t // 舵向电机
    {
        int16_t now_pos[4];
        void (*fSetPos)(float pos[4]); // 舵轮角度环控制的函数指针
        uint8_t can_send_flag;
    } SW_SteerMotor_t;

    typedef struct SW_DriveMotor_t // 主驱动电机
    {
        uint32_t can_id[4];   // can消息id
        float target_duty[4]; // 占空比
        float target_curr[4]; // 电流
        float target_rpm[4];  // 转速
        uint8_t motor_mode;
        uint8_t can_send_flag;
        void (*fSetSpeed)(float speed[4]);
        void (*fHandbrake)(float hand_brake_current);
    } SW_DriveMotor_t;

    //=================================全局变量=================================

    extern int SW_InitCalibration_BreakFlag;
    extern uint8_t SW_PrintMotorStatus_Flag;

    //=================================函数声明=================================

    void SW_MotorInit(struct SW_DriveMotor_t *driver_motor, struct SW_SteerMotor_t *steer_motor);
    void SW_DriveMotors_SetSpeed(float vel[4]);
    void SW_DriveMotors_LimitSpeed(float rpm[4]);
    void SW_SteerMotors_SetPos(float target_pos[]);
    void SW_SteerMotors_LimitPos(float pos[4]);
    void SW_SteerMotors_GetAngle();
    void SW_InitCalibration();
    float SW_Speed2eRPM(float speed);
    void SW_PrintMotorStatus();
    void SW_PrintHallSwitchStatus(void);
    void SW_DriveMotors_SetCurr(float current[4]);
    void SW_DriveMotors_SetDuty(float duty[4]);
    void SW_DriveMotors_Handbrake(float hand_brake_current);
#ifdef __cplusplus
}
#endif
#endif

#endif