/**
 * @file omni_chassis.c
 * @author simon
 * @brief 横辊子全向轮运动学相关
 * @version 0.1
 * @date 2021-04-25
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "base_chassis.h"
#include "omni_chassis.h"
#include "chassis_common_config.h"
#include "can_utils.h"
#include "utils.h"
#include "handle.h"
#include "motor_driver.h"

#ifdef USE_CHASSIS_OMNI

OmniChassis_t OmniChassis;
static float InstallAngle[4] = {-2.3561945625, 2.3561945625, -0.7853981875, 0.7853981875}; // 车轮正方形与底盘坐标系x轴的夹角/rad
static CAN_Message_u MotorRpmCANMsg[4];
char MotorCANSendFlag = 0; // 需要在定时中断中置1，控制发送频率

/**
 * @brief 横辊子全向轮运动学解算
 * 
 * @param target_speed m/s
 * @param target_dir rad
 * @param target_omega rad/s
 */
void OmniChassis_Move(float target_speed, float target_dir, float target_omega)
{
    LIMIT(target_speed, DRIVE_WHEEL_MAX_SPEED);

    float absolute_angle_offset = 0; // 使用绝对坐标时，根据全场定位测得的偏航角进行补偿
    if (OmniChassis.base->pos_mode == POS_MODE_ABSOLUTE)
    {
        absolute_angle_offset = OmniChassis.base->posture_status.yaw - 1.5708; // 车头初始方向90°
    }
    target_dir -= absolute_angle_offset;

    // 将目标速度向量投影到每个驱动轮上
    float speed_out_0 = (target_speed * cos(InstallAngle[0] - target_dir));
    float speed_out_1 = (target_speed * cos(InstallAngle[1] - target_dir));
    float speed_out_2 = (target_speed * cos(InstallAngle[2] - target_dir));
    float speed_out_3 = (target_speed * cos(InstallAngle[3] - target_dir));

    // 与角速度在每个轮上的线速度合成
    float diagonal_length = Wheel2ChassisCenter;
    float motor_speed[4];
    motor_speed[0] = speed_out_0 + target_omega * diagonal_length;
    motor_speed[1] = speed_out_1 + target_omega * diagonal_length;
    motor_speed[2] = speed_out_2 + target_omega * diagonal_length;
    motor_speed[3] = speed_out_3 + target_omega * diagonal_length;

    if (OmniChassis.motor_can_flag)
    {
        for (int i = 0; i < 4; i++)
        {
            MotorRpmCANMsg[i].in[0] = 1;
            MotorRpmCANMsg[i].in[1] = (int)(motor_speed[i] / DRIVE_WHEEL_RADIUS * 30.0f / PI); //m/s转为rpm
            CAN_SendMsg(OmniChassis.drive_motors[i].can_id, &MotorRpmCANMsg[i]);
            HAL_Delay(1);
        }
        OmniChassis.motor_can_flag = 0;
    }
}

/**
 * @brief 三轮横辊子全向轮运动学
 * 
 * @param target_speed 
 * @param target_dir 
 * @param target_omega 
 */
void OmniChassis_Move_3Wheel(float target_speed, float target_dir, float target_omega)
{

}

/**
 * @brief 初始化函数
 * 
 */
void OmniChassis_Init()
{
    BaseChassis_Init();
    OmniChassis.base = &BaseChassis;
    OmniChassis.base->fChassisMove = Omni_ChassisMove;
    for (int i = 0; i < 4; i++)
    {
        // VESC_Init(&OmniChassis.drive_motors[i],i + OMNI_DRIVE_MOTOR_BASE_CANID,(VESCMode)VESC_RPM); // 若使用本杰明驱动器
        MotorDriver_Init(&OmniChassis.drive_motors[i], i + OMNI_DRIVE_MOTOR_BASE_CANID, MTR_CTRL_RPM); // 若使用BUPT自制驱动卡
    }
    OmniChassis.motor_can_flag = 0;
}

/**
 * @brief 横辊子全向轮状态机
 * 
 */
void OmniChassis_Exe()
{
    Chassis_UpdatePostureStatus();
    switch (OmniChassis.base->ctrl_mode)
    {
    case CTRL_MODE_NONE:
        break;
    case CTRL_MODE_HANDLE:
        Handle_Exe();
        break;
    case CTRL_MODE_CMD:
        OmniChassis.base->target_speed = CMD_TargetSpeed;
        OmniChassis.base->target_dir = __ANGLE2RAD(CMD_TargetDir);
        OmniChassis.base->target_omega = CMD_TargetOmega;
        break;
    case CTRL_MODE_TRACK:
        OmniChassis.base->pos_mode = POS_MODE_ABSOLUTE;
        Chassis_TrackPathSets(OmniChassis.base->track_status.path_index);
        break;
    case CTRL_MODE_TUNING:
        Chassis_YawTuning(CMD_TargetYaw);
        break;
    default:
        break;
    }
    OmniChassis_Move(OmniChassis.base->target_speed, OmniChassis.base->target_dir, OmniChassis.base->target_omega); // 底层执行函数
}

#endif