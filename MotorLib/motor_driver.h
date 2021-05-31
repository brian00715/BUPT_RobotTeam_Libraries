#ifndef _MOTOR_DRIVER_H
#define _MOTOR_DRIVER_H
#include "can_utils.h"
#include "main.h"
#include "vesc_can.h"

typedef struct CANSendFlag_t
{
    unsigned m2006 : 1;
    unsigned vesc : 1;
} CANSendFlag_t; // 电调CAN消息发送标志位

typedef enum VESCMode
{
    VESC_DUTY = 0,
    VESC_CURRENT,
    VESC_RPM,
    VESC_POSITION
} VESCMode;

typedef struct VESCDriver_t
{
    uint8_t id;            // 本杰明电调的can消息id
    VESCMode mode;         // 模式（速度环、电流环等）
    float target_duty;     // 占空比
    float target_current;  // 电流
    float target_rpm;      // 转速
    float target_position; // 位置
    float now_duty;
    float now_current;
    float now_rpm;
    float now_position;
} VESCDriver_t; // 本杰明驱动器结构体

typedef enum DriverMode
{
    DRIVER_DUTY = 0,
    DRIVER_CURRENT,
    DRIVER_RPM,
    DRIVER_POSITION
} DriverMode;

typedef struct MotorDriver_t
{
    uint8_t can_id;        // can消息id
    DriverMode mode;       // 模式（速度环、电流环等）
    float target_duty;     // 占空比
    float target_current;  // 电流
    float target_rpm;      // 转速
    float target_position; // 位置
    float now_duty;
    float now_current;
    float now_rpm;
    float now_position;
} MotorDriver_t; // 电机驱动器抽象结构体

void VESC_Init(VESCDriver_t *vesc, uint8_t id, VESCMode mode);
void VESC_Exe();
void m2006_Exe();
void VESC_RxHandler(CANMsg *pRxMsg);
void VESC_PrintInfo(VESCDriver_t *vesc);
void MotorDriver_Init(MotorDriver_t *motor_driver, uint8_t can_id, DriverMode mode);
extern VESCDriver_t vesc;
extern int VESC_StatusBag_Flag;
extern int VESC_SwitchPrintInfo_Flag;
extern int VESC_SwitchStopByAngle_Flag;

#endif //_MOTOR_DRIVER_H