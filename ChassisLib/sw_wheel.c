/*******************************************************************
 * @brief 舵轮电机配置相关
 * @author Simon
 * @date 2020/11/
 ******************************************************************/
#ifdef USE_RUDDER_CHASSIS
#include "sw_wheel.h"
extern struct SW_Chassis_t SW_Chassis;

//======================================private==========================================

//=======================================END=============================================

//========================================public========================================

void SW_MotorInit(struct SW_DriveMotor_t *driver_motor, struct SW_SteerMotor_t *steer_motor)
{
    for (int i = 0; i < 5; i++)
    {

        driver_motor->rpm[i] = 0;
        driver_motor->id[i] = i + VESC_ID_BASE; // 设置本杰明电调ID
        steer_motor->now_pos[i] = 0;
        MotorOn(CAN1, i);
        PosLoopCfg(CAN1, i, 0, 0, STEER_WHEEL_MAX_SPEED); // 使用位置环模式
    }
    // 注册函数
    steer_motor->setPos = SW_SteerMotors_SetPos;
    driver_motor->setRPM = SW_DriveMotors_SetRpm;
}

int SW_InitCalibration_BreakFlag = 0;
/**
 * @brief 校正舵轮到初始位置，位置找正之前在此函数中阻塞
 * @note 调用时需要将舵轮统一转到距离正确起点 顺时针 偏移一些的角度
 */
void SW_InitCalibration()
{
    if (SW_InitCalibration_BreakFlag)
    {
        SW_InitCalibration_BreakFlag = 0;
        return;
    }
    uprintf("--SW: Start Calibrating the SteerWheel!\r\n");
    int16_t speed[4] = {400, 400, 400, 400};
    int16_t stop[4] = {0};
    for (int i = 0; i < 4; i++)
    {
        VelLoopCfg(CAN1, i + 1, 0, 0); // 配置速度环
    }
    DJI_velCtrAll(speed); // 开始旋转
    int16_t hall_switch_flag[4] = {0};
    while (1) // 四个都到位才退出循环
    {
        if (HAL_GPIO_ReadPin(HallSwitch1_GPIO_Port, HallSwitch1_Pin))
        {
            DJI_VelCrl(CAN1, 1, 0);
            hall_switch_flag[0] = 1;
        }
        else
        {
            DJI_VelCrl(CAN1, 1, 400);
            hall_switch_flag[0] = 0;
        }

        if (HAL_GPIO_ReadPin(HallSwitch2_GPIO_Port, HallSwitch2_Pin))
        {
            DJI_VelCrl(CAN1, 2, 0);
            hall_switch_flag[1] = 1;
        }
        else
        {
            DJI_VelCrl(CAN1, 2, 400);
            hall_switch_flag[1] = 0;
        }

        if (HAL_GPIO_ReadPin(HallSwitch3_GPIO_Port, HallSwitch3_Pin))
        {
            DJI_VelCrl(CAN1, 3, 0);
            hall_switch_flag[2] = 1;
        }
        else
        {
            DJI_VelCrl(CAN1, 3, 400);
            hall_switch_flag[2] = 0;
        }
        if (HAL_GPIO_ReadPin(HallSwitch4_GPIO_Port, HallSwitch4_Pin))
        {
            DJI_VelCrl(CAN1, 4, 0);
            hall_switch_flag[3] = 1;
        }
        else
        {
            DJI_VelCrl(CAN1, 4, 400);
            hall_switch_flag[3] = 0;
        }
        if (Sum_int16ar(hall_switch_flag, 4) == 4)
        {
            break;
        }
    }
    DJI_velCtrAll(stop);
    SW_Chassis.ctrl_mode = SW_MODE_NONE;
    uprintf("--SW: SteerWheel Calibration Done!\r\n");
    SW_PostureStatusInit();
    SW_Handle_Init();
    for (int i = 0; i < 4; i++)
    {
        MotorOn(CAN1, i + 1);                                 // 使驱动板认为的瞬时角度值为0
        PosLoopCfg(CAN1, i + 1, 0, 0, STEER_WHEEL_MAX_SPEED); // 更改到位置环模式
    }
}

void SW_PrintHallSwitchStatus(void)
{
    if (TimeFlag_50ms)
    {
        uprintf("--HallSwitch|[0]:%d [1]:%d [2]:%d [3]:%d\r\n",
                HAL_GPIO_ReadPin(HallSwitch1_GPIO_Port, HallSwitch1_Pin),
                HAL_GPIO_ReadPin(HallSwitch2_GPIO_Port, HallSwitch2_Pin),
                HAL_GPIO_ReadPin(HallSwitch3_GPIO_Port, HallSwitch3_Pin),
                HAL_GPIO_ReadPin(HallSwitch4_GPIO_Port, HallSwitch4_Pin));
    }
}

int SW_SendCanMsg_Driver = 0; // 控制5ms发送一次can消息
int SW_SendCanMsg_Steer = 0;
/**
 * @brief 发送CAN报文，同时设置4个舵轮的舵向
 * @param target_pos 目标角度/rad 需要转为degree发给DJI电调
 */
void SW_SteerMotors_SetPos(float target_pos[])
{
    if (!SW_SendCanMsg_Steer) // 控制发送周期
        return;
    // for (int i = 0; i < 4; i++)
    // {
    //     target_pos[i] += SW_ZeroPos[i]; // 补偿零位校正
    // }
    SW_SteerMotors_LimitPos(target_pos);
    static int16_t target_pos_int[4];
    for (int i = 0; i < 4; i++)
    {
        target_pos_int[i] = (int16_t)RAD2ANGLE(target_pos[i]);
    }
    DJI_posCtrlAll(target_pos_int);
    SW_SendCanMsg_Steer = 0;
}

/**
 * @brief 向驱动轮电调发送速度命令
 * 
 * @param vel 单位m/s
 * @note 需要将vel转为电转速 
 */
void SW_DriveMotors_SetRpm(float vel[4])
{
    if (!SW_SendCanMsg_Driver) // 控制发送周期
        return;
    for (int i = 0; i < 4; i++)
    {
        vel[i] = SW_Speed2eRPM(vel[i]); // >>>请根据不同电机进行换算<<<
        comm_can_set_rpm(SW_Chassis.DriveMotors->id[i], vel[i]);
        if (3 == i || 2 == i)
            HAL_Delay(1); // 发送间隔1ms，否则最后一个发不出去
    }
    SW_SendCanMsg_Driver = 0;
}

/**
 * @brief 将底盘速度m/s转为电机电转速
 * 
 * @param speed m/s
 * @return float eRPM=RPM*Pole Pairs
 */
float SW_Speed2eRPM(float speed)
{
    return ((speed * 30) / (PI * DRIVE_WHEEL_RADIUS)) * DRIVE_MOTOR_POLE_PARIS * DRIVE_WHEEL_REDUCTION_RATIO;
}

void SW_SteerMotors_LimitPos(float pos[4])
{
    for (int i = 0; i < 4; i++)
    {
        LIMIT_FROM_TO(pos[i], STEER_WHEEL_MIN_POS, STEER_WHEEL_MAX_POS);
    }
}

/** 
 * @brief  查询舵向电机角度
 * @note 驱动板发回0x281的反馈CAN消息，数据读到全局结构体Robomaster_Motor中
 **/
void SW_SteerMotors_GetAngle()
{
    for (int i = 0; i < 4; i++)
    {
        DJI_ReadActualPos(CAN1, i + 1);
    }
}

extern RM_MotorStatus_t RM_MotorStatus[4];
int PrintMotorStatus_Flag = 0;
void SW_PrintMotorStatus()
{
    if (!(PrintMotorStatus_Flag && TimeFlag_20ms))
        return;
    uprintf("--DJIMotor|");

    uprintf("pos>");
    for (int i = 0; i < 4; i++)
    {
        //除以减速比得到舵轮的实际位置
        uprintf("[%d]%.2f ", i, RAD2ANGLE(RM_MotorStatus[i].pos / STEER_WHEEL_REDUCTION_RATIO));
    }
    uprintf("<\r\n");
}

/**
 * @brief 限制速度绝对值大小
 * 
 * @param speed 
 * @note 注意最小速度不能太大，否则底盘不会静止
 */
void SW_DriveMotors_LimitSpeed(float speed[4])
{
    for (int i = 0; i < 4; i++)
    {
        if (speed[i] < 0)
        {
            speed[i] *= -1;
            LIMIT_FROM_TO(speed[i], DRIVE_WHEEL_MIN_SPEED, DRIVE_WHEEL_MAX_SPEED);
            speed[i] *= -1;
            continue;
        }
        LIMIT_FROM_TO(speed[i], DRIVE_WHEEL_MIN_SPEED, DRIVE_WHEEL_MAX_SPEED);
    }
}
//=======================================END=============================================
#endif