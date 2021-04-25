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

typedef struct VESCMotor_t
{
    uint8_t id;            // 本杰明电调的can消息id
    char mode;             // 模式（速度环、电流环等）
    float target_duty;     // 占空比
    float target_current;  // 电流
    float target_rpm;      // 转速
    float target_position; // 位置
    float now_duty;
    float now_current;
    float now_rpm;
    float now_position;
} VESCMotor_t;


void VESC_Init(VESCMotor_t *vesc, uint8_t id, char mode);
void VESC_Exe();
void m2006_Exe();
void VESC_RxHandler(can_msg *pRxMsg);
void VESC_PrintInfo(VESCMotor_t *vesc);

extern VESCMotor_t vesc;
extern int VESC_StatusBag_Flag;
extern int VESC_SwitchPrintInfo_Flag;
extern int VESC_SwitchStopByAngle_Flag;

#endif //_MOTOR_DRIVER_H