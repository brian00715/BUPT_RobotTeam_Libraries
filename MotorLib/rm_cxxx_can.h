#ifndef RM_CXXX_CAN_H_
#define RM_CXXX_CAN_H_
#ifdef __cplusplus
extern "C"
{
#endif

#include "motor_driver.h"
#ifdef USE_MTR_DRIVER_RM_CXXX


#include "utils.h"
#include "can_utils.h"

  /*Struct Area*/
  typedef enum
  {
    _M2006,
    _M3508
  } ROBOMASTER_TYPE;

  typedef struct
  {
    float target_duty;       // 占空比
    float target_current;    // 电流
    int16_t target_rpm;      // 转速
    int16_t target_position; // 位置（线数）

    int16_t now_rpm;
    float now_current;
    int16_t now_position;
    int16_t now_duty;

    uint8_t hall;
    uint16_t last_position; //abs angle range:[0,8191]
    uint16_t offset_position;
    int32_t round_cnt;
    int32_t total_angle;

    ROBOMASTER_TYPE type;
  } DJIMotor_t; // 主控板直接与大疆电调连接时使用的状态结构体

  /*Variable Area*/
  extern int ver_slide_error;
  extern int robomaster_flag;
  extern int Robomaster_RPMValue[4];
  extern int RM_PrintInfo_Flag;
  extern int Robomaster_PositionOffset;
  extern int RM_RPMControl_Flag;
  extern int RM_PosControl_Flag;
  extern uint32_t Robomaster_OpenAngleControl_Flag; // CMD控制是否启动转角停止，置1开启
  extern uint32_t Robomaster_TargetOffsetAngle;     // CMD指定目标转角变化量

  extern DJIMotor_t DJIMotor[4];

  extern PID_s RM_Speed_PID[4];
  extern PID_s RM_Position_PID[4];

  /*Function Declare*/
  void can_robomaster_rcv_1(CAN_Message_u *pRxMsg);
  void can_robomaster_rcv_2(CAN_Message_u *pRxMsg);
  void Robomaster_RPMControl();
  void Robomaster_PositionControl();
  void Robomaster_StopByAngle(int index);
  float robomaster_position_pid_control(int id);
  float robomaster_speed_pid_control(int id);
  void robomaster_set_current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
  void Robomaster_PrintInfo(int index);
  void M3508_init(int id);
  void M2006_init(int id);

#ifdef __cplusplus
}
#endif

#endif 

#endif