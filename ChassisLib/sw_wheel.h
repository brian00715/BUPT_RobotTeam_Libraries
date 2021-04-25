#ifndef __STEER_WHEEL_H_
#define __STEER_WHEEL_H_

#ifdef USE_RUDDER_CHASSIS

#ifdef __cplusplus
extern "C"
{
#endif
//=================================头文件包含=================================
#include "main.h"
#include "simplelib.h"
#include "can_utils.h"
#include "dji_ctr.h"
#include "motor_driver.h"
#include "chassis_utils.h"
#include "sw_config.h"
    //=================================宏和枚举类=================================

    typedef enum
    {
        DUTY = 0,
        CURRENT,
        RPM,
        POS
    } VESC_MODE;

    //=================================结构体=================================

    struct SW_SteerMotor_t // 舵向电机m2006
    {
        int16_t now_pos[4];
        void (*setPos)(float pos[4]); // 舵轮角度环控制的函数指针
    };

    struct SW_DriveMotor_t // 主驱动电机（本杰明控制）
    {
        uint8_t id[4];    // 本杰明电调的can消息id
        float duty[4];    // 占空比
        float current[4]; // 电流
        float rpm[4];     // 转速
        uint16_t pos[4];  // 位置
        void (*setRPM)(float speed[4]);
    };

    //=================================全局变量=================================

    extern int SW_SendCanMsg_Driver;
    extern int SW_SendCanMsg_Steer;
    extern int SW_InitCalibration_BreakFlag;
    extern float SW_ZeroPos[4];

    //=================================函数声明=================================

    void SW_MotorInit(struct SW_DriveMotor_t *driver_motor, struct SW_SteerMotor_t *steer_motor);
    void SW_DriveMotors_SetRpm(float vel[4]);
    void SW_DriveMotors_LimitSpeed(float rpm[4]);
    void SW_SteerMotors_SetPos(float target_pos[]);
    void SW_SteerMotors_LimitPos(float pos[4]);
    void SW_SteerMotors_GetAngle();
    void SW_InitCalibration();
    float SW_Speed2eRPM(float speed);
    void SW_PrintMotorStatus();
    void SW_PrintHallSwitchStatus(void);
#ifdef __cplusplus
}
#endif
#endif

#endif