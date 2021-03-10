/**
 * @file rm_cxxx_can.c
 * @author simon
 * @brief 大疆C610/620电调CAN通信协议相关
 * @version 0.1
 * @date 2021-07-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */


#include "rm_cxxx_can.h"
#ifdef USE_MTR_DRIVER_RM_CXXX

DJIMotor_t DJIMotor[4];                            // 大疆电机的参数结构体，包含转角、速度等参数，在can_robomaster_rcv_1中更新数值
int Robomaster_RPMValue[4] = {50, 0, 0, 0};        // 4个大疆电机的速度设定值数组
int Robomaster_PositionValue[4] = {1000, 0, 0, 0}; // 4个大疆电机的位置角设定值数组
int Robomaster_PositionOffset = 0;
// 大疆电机跑速度环的PID参数
PID_s RM_Speed_PID[4] = {1.4, 0.9, 0.6, 0, 0, 5000, 0, 0.005}; // = {1.4,0.9,0.6,0,0,5000,0,0.005};
// 大疆电机跑位置环的PID参数
PID_s RM_Position_PID[4] = {0.13, 0, 0.082, 0, 0, 5000, 0, 0.005}; // = {0.13,0,0.082,0,0,5000,0,0.005};
int robomaster_flag = 0;
int ver_slide_error = 0;
int RM_PrintInfo_Flag = 0;
int RM_RPMControl_Flag = 0;
int RM_PosControl_Flag = 0;

/*接收1号电调发送的反馈CAN消息*/
void can_robomaster_rcv_1(CAN_Message_u *pRxMsg)
{
  //pRxMsg->StdId
  static int first_flag = 1;
  if (first_flag == 1)
  {
    DJIMotor[0].now_position = (uint16_t)(pRxMsg->ui8[0] << 8 | pRxMsg->ui8[1]);
    DJIMotor[0].offset_position = DJIMotor[0].now_position;
    first_flag = 0;
    DJIMotor[0].round_cnt = 0;
    return;
  }
  DJIMotor[0].last_position = DJIMotor[0].now_position;
  DJIMotor[0].now_position = (uint16_t)(pRxMsg->ui8[0] << 8 | pRxMsg->ui8[1]);
  DJIMotor[0].now_rpm = (int16_t)(pRxMsg->ui8[2] << 8 | pRxMsg->ui8[3]);
  DJIMotor[0].now_current = (pRxMsg->ui8[4] << 8 | pRxMsg->ui8[5]) * 5.f / 16384.f;

  if (DJIMotor[0].now_position - DJIMotor[0].last_position > 4096)
    DJIMotor[0].round_cnt--;
  else if (DJIMotor[0].now_position - DJIMotor[0].last_position < -4096)
    DJIMotor[0].round_cnt++;
  DJIMotor[0].total_angle = DJIMotor[0].round_cnt * 8192 + DJIMotor[0].now_position - DJIMotor[0].offset_position;
  // if (Robomaster_PrintInfo_Flag == 1)
  // {
  //   uprintf("--robomaster[0] info:\r\n");
  //   uprintf("  angle:%d speed_rpm:%d current:%d\r\n",
  //           robomaster[0].now_position, robomaster[0].now_rpm, robomaster[0].now_current);
  // }
}

/*接收2号电调发送的反馈CAN消息*/
void can_robomaster_rcv_2(CAN_Message_u *pRxMsg)
{
  //pRxMsg->StdId
  //锟斤拷锟斤拷id锟斤拷没锟斤拷
  static int first_flag = 1;
  if (first_flag == 1)
  {
    DJIMotor[1].now_position = (uint16_t)(pRxMsg->ui8[0] << 8 | pRxMsg->ui8[1]);
    DJIMotor[1].offset_position = DJIMotor[1].now_position;
    first_flag = 0;
    DJIMotor[1].round_cnt = 0;
    return;
  }
  DJIMotor[1].last_position = DJIMotor[0].now_position;
  DJIMotor[1].now_position = (uint16_t)(pRxMsg->ui8[0] << 8 | pRxMsg->ui8[1]);
  DJIMotor[1].now_rpm = (int16_t)(pRxMsg->ui8[2] << 8 | pRxMsg->ui8[3]);
  DJIMotor[1].now_current = (pRxMsg->ui8[4] << 8 | pRxMsg->ui8[5]) * 5.f / 16384.f;

  if (DJIMotor[1].now_position - DJIMotor[1].last_position > 4096)
    DJIMotor[1].round_cnt--;
  else if (DJIMotor[1].now_position - DJIMotor[1].last_position < -4096)
    DJIMotor[1].round_cnt++;
  DJIMotor[1].total_angle = DJIMotor[1].round_cnt * 8192 + DJIMotor[1].now_position - DJIMotor[1].offset_position;
}

/*跑速度环*/
void Robomaster_RPMControl()
{
  if (RM_RPMControl_Flag == 0)
  {
    return;
  }
  float speed_out[4];
  for (int i = 0; i < 4; i++)
  {
    speed_out[i] = robomaster_speed_pid_control(i); // 根据设定速度得出应该发送的电流值
  }
  robomaster_set_current((int16_t)speed_out[0], (int16_t)speed_out[1], (int16_t)speed_out[2], (int16_t)speed_out[3]);
}

/*跑位置环*/
void Robomaster_PositionControl()
{
  if (RM_PosControl_Flag == 0)
  {
    return;
  }
  float pos_out[4];
  for (int i = 0; i < 4; i++)
  {
    pos_out[i] = robomaster_position_pid_control(i); // 根据设定速度得出应该发送的电流值
    if (pos_out[i] > 1000)                           // 限制电流大小
    {
    }
  }
}

/**
 * @brief 大疆电机跑速度环的pid计算函数
 * @param id 要设置的电调can id
 * @return 当前应发送给电调的速度值
 **/
float robomaster_speed_pid_control(int id)
{
  float speed_out = 0;
  speed_out = PID_Release(&RM_Speed_PID[id], Robomaster_RPMValue[id], (float)DJIMotor[id].now_rpm);
  //speed_out=PID_Release(&Robomaster_Speed_PID[id],(float)robomaster[id].target_speed,(float)robomaster[id].speed_rpm);
  return speed_out;
}

/**
 * @brief 大疆电机跑位置环的pid计算函数
 * @param id 要设置的电调can id
 * @return 当前应发送给电调的速度值
 **/
float robomaster_position_pid_control(int id)
{
  float speed_out = 0;
  float position_out = 0;
  position_out = PID_Release(&RM_Position_PID[id],
                             (float)DJIMotor[id].target_position + ver_slide_error, (float)DJIMotor[id].total_angle);
  speed_out = PID_Release(&RM_Speed_PID[id], position_out, (float)DJIMotor[id].now_rpm);
  return speed_out;
}

/**
 * @brief 设置大疆电机电流
 * @param iq1 电流值/mA
 **/
void robomaster_set_current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
  uint8_t Data[8];
  Data[0] = (iq1 >> 8);
  Data[1] = iq1;
  Data[2] = (iq2 >> 8);
  Data[3] = iq2;
  Data[4] = iq3 >> 8;
  Data[5] = iq3;
  Data[6] = iq4 >> 8;
  Data[7] = iq4;
  CAN_Message_u dji_data;
  for (int i = 0; i <= 7; i++)
  {
    dji_data.ui8[i] = Data[i];
  }
  CAN_SendMsg(0x200, &dji_data); // 前4个电机的can_id为0x200
}

/**
 * @brief 打印大疆电机信息
 * @param index 电调序号
 *        @arg 0 1 2 3
 **/
void Robomaster_PrintInfo(int index)
{
  if (RM_PrintInfo_Flag == 1)
  {
    // uprintf("--robomaster[%d] info:\r\n", index);
    uprintf("  now_angle:%d total_angle:%d speed_rpm:%d current:%d\r\n",
            (uint32_t)DJIMotor[index].now_position * 360 / 8192, (uint32_t)DJIMotor[index].total_angle * 360 / 8192,
            DJIMotor[index].now_rpm, DJIMotor[index].now_current);
  }
}

uint32_t Robomaster_OpenAngleControl_Flag = 0; // CMD控制是否启动转角停止，置1开启
uint32_t Robomaster_TargetOffsetAngle = 0;     // CMD指定目标转角变化量，单位度（不是弧度）
static uint32_t now_angle = 0;
static uint32_t robomaster_origin_angle = 0;
static int first_flag = 0;
/**
 * @brief 控制大疆电机转过一定角度后停止,需要在m2006_exm2006_Exe
 *        目标转角变化量是通过CMD指定的全局变量
 * @param index 大疆电调编号
 *    @arg 0 1 2 3
 * @return none
 **/
void Robomaster_StopByAngle(int index)
{
  if (Robomaster_OpenAngleControl_Flag == 0)
  {
    return;
  }

  if (first_flag == 0) // 记录初始角度值
  {
    robomaster_origin_angle = (uint32_t)DJIMotor[index].total_angle * 360 / 8192;
    first_flag = 1;
  }
  else
  {
    now_angle = (uint32_t)DJIMotor[index].total_angle * 360 / 8192;
    int offset_position = now_angle - robomaster_origin_angle; // ！！！这里假定角度值不会溢出！！！
    if (offset_position >= Robomaster_TargetOffsetAngle)
    {
      robomaster_set_current(0, 0, 0, 0); // 立即执行
      DJIMotor[index].target_current = 0; // 将mm2006_Exe)中的执行电流置0
      /*TODO:后期可以把上面两行改成发给本杰明电调停止信号*/
      robomaster_origin_angle = 0;
      offset_position = 0;
      first_flag = 0;
      Robomaster_OpenAngleControl_Flag = 0; // 如果还需使用，则通过CMD再次置1
    }
  }
}

void M2006_init(int id)
{
  PID_Reset(&RM_Speed_PID[id]);
  PID_Reset(&RM_Position_PID[id]);
  RM_Speed_PID[id].Kp = 1.4;
  RM_Speed_PID[id].Ki = 0.9;
  RM_Speed_PID[id].Kd = 0.6;
  RM_Speed_PID[id].int_max = 5000;
  RM_Speed_PID[id].int_duty = 0.005;

  RM_Position_PID[id].Kp = 0.13;
  RM_Position_PID[id].Ki = 0;
  RM_Position_PID[id].Kd = 0.082;
  RM_Position_PID[id].int_max = 5000;
  RM_Position_PID[id].int_duty = 0.005;
}

void M3508_init(int id)
{
  PID_Reset(&RM_Speed_PID[id]);
  PID_Reset(&RM_Position_PID[id]);
  RM_Speed_PID[id].Kp = 1.6;
  RM_Speed_PID[id].Ki = 0.52;
  RM_Speed_PID[id].Kd = 0.6;
  RM_Speed_PID[id].int_max = 5000;
  RM_Speed_PID[id].int_duty = 0.005;

  RM_Position_PID[id].Kp = 0.1;
  RM_Position_PID[id].Ki = 0;
  RM_Position_PID[id].Kd = 0.8;
  RM_Position_PID[id].int_max = 5000;
  RM_Position_PID[id].int_duty = 0.005;
}

#endif