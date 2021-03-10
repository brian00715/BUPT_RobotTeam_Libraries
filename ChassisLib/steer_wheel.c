/*******************************************************************
 * @brief 舵轮电机配置相关
 * @author Simon
 * @date 2020/11/
 ******************************************************************/
#include "steer_wheel.h"
#include "rudder_chassis.h"
#include "odrive_can.h"
#include "vesc_can.h"
#ifdef USE_CHASSIS_RUDDER
extern RudderChassis_s RudderChassis;

#ifdef DRIVE_MTR_USE_ODRIVE
ODrive_t odrive[2];
#endif

//======================================private==========================================

//=======================================END=============================================

//========================================public========================================

void SW_MotorInit(struct SW_DriveMotor_t *driver_motor, struct SW_SteerMotor_t *steer_motor)
{
    for (int i = 0; i < 4; i++)
    {

        driver_motor->target_rpm[i] = 0;
        driver_motor->target_curr[i] = 0;
        driver_motor->target_duty[i] = 0;
        driver_motor->motor_mode = MTR_CTRL_RPM;
        driver_motor->can_id[i] = i + VESC_ID_BASE; // 设置本杰明电调ID
        steer_motor->now_pos[i] = 0;

        DJIBoard_MotorOn(CAN1, i);
        DJIBoard_PosLoopCfg(CAN1, i, 0, 0, STEER_WHEEL_MAX_SPEED); // 舵向轮使用位置环模式
    }
    // 注册函数
    steer_motor->fSetPos = SW_SteerMotors_SetPos;
    driver_motor->fSetSpeed = SW_DriveMotors_SetSpeed;
    driver_motor->fHandbrake = SW_DriveMotors_Handbrake;
#ifdef DRIVE_MTR_USE_ODRIVE
    // 配置ODrive模式
    odrive[0].axis0.set_state.input_mode = ODRIVE_INPUT_MODE_VEL_RAMP;
    odrive[0].axis1.set_state.input_mode = ODRIVE_INPUT_MODE_VEL_RAMP;
    odrive[0].axis0.set_state.control_mode = ODRIVE_CTRL_MODE_VEL;
    odrive[0].axis1.set_state.control_mode = ODRIVE_CTRL_MODE_VEL;
    odrive[1].axis0.set_state.input_mode = ODRIVE_INPUT_MODE_VEL_RAMP;
    odrive[1].axis1.set_state.input_mode = ODRIVE_INPUT_MODE_VEL_RAMP;
    odrive[1].axis0.set_state.control_mode = ODRIVE_CTRL_MODE_VEL;
    odrive[1].axis1.set_state.control_mode = ODRIVE_CTRL_MODE_VEL;
    ODrive_CANSendMsg(&hcan1, &odrive[0], AXIS_0, ODRIVE_MSG_SET_CONTROLLER_MODES);
    ODrive_CANSendMsg(&hcan1, &odrive[0], AXIS_1, ODRIVE_MSG_SET_CONTROLLER_MODES);
    ODrive_CANSendMsg(&hcan1, &odrive[1], AXIS_0, ODRIVE_MSG_SET_CONTROLLER_MODES);
    ODrive_CANSendMsg(&hcan1, &odrive[1], AXIS_1, ODRIVE_MSG_SET_CONTROLLER_MODES);
#endif
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
        DJIBoard_VelLoopCfg(CAN1, i + 1, 0, 0); // 配置速度环
    }
    DJIBoard_VelCtrlAll(speed); // 开始旋转
    int16_t hall_switch_flag[4] = {0};
    while (1) // 四个都到位才退出循环
    {
        if (HAL_GPIO_ReadPin(HallSwitch1_GPIO_Port, HallSwitch1_Pin))
        {
            DJIBoard_VelCrl(CAN1, 1, 0);
            hall_switch_flag[0] = 1;
        }
        else
        {
            DJIBoard_VelCrl(CAN1, 1, 400);
            hall_switch_flag[0] = 0;
        }

        if (HAL_GPIO_ReadPin(HallSwitch2_GPIO_Port, HallSwitch2_Pin))
        {
            DJIBoard_VelCrl(CAN1, 2, 0);
            hall_switch_flag[1] = 1;
        }
        else
        {
            DJIBoard_VelCrl(CAN1, 2, 400);
            hall_switch_flag[1] = 0;
        }

        if (HAL_GPIO_ReadPin(HallSwitch3_GPIO_Port, HallSwitch3_Pin))
        {
            DJIBoard_VelCrl(CAN1, 3, 0);
            hall_switch_flag[2] = 1;
        }
        else
        {
            DJIBoard_VelCrl(CAN1, 3, 400);
            hall_switch_flag[2] = 0;
        }
        if (HAL_GPIO_ReadPin(HallSwitch4_GPIO_Port, HallSwitch4_Pin))
        {
            DJIBoard_VelCrl(CAN1, 4, 0);
            hall_switch_flag[3] = 1;
        }
        else
        {
            DJIBoard_VelCrl(CAN1, 4, 400);
            hall_switch_flag[3] = 0;
        }
        double result;
        __SUM_OF_AR(hall_switch_flag, 4, result);
        if ((int)result == 4)
        {
            break;
        }
    }
    DJIBoard_VelCtrlAll(stop);
    RudderChassis.base->ctrl_mode = CTRL_MODE_NONE;
    uprintf("--SW: SteerWheel Calibration Done!\r\n");
    Chassis_PostureStatusInit();
    Handle_Init();
    for (int i = 0; i < 4; i++)
    {
        DJIBoard_MotorOn(CAN1, i + 1);                                 // 使驱动板认为的瞬时角度值为0
        DJIBoard_PosLoopCfg(CAN1, i + 1, 0, 0, STEER_WHEEL_MAX_SPEED); // 更改到位置环模式
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

/**
 * @brief 发送CAN报文，同时设置4个舵轮的舵向
 * @param target_pos 目标角度/rad 假定已经乘过减速比。需要转为degree发给DJI电调
 */
void SW_SteerMotors_SetPos(float target_pos[])
{
    if (!RudderChassis.SteerMotors.can_send_flag) // 控制发送周期
        return;
    SW_SteerMotors_LimitPos(target_pos);
    static int16_t target_pos_int[4];
    for (int i = 0; i < 4; i++)
    {
        target_pos_int[i] = (int16_t)__RAD2ANGLE(target_pos[i]);
    }
    DJIBoard_PosCtrlAll(target_pos_int);
    RudderChassis.SteerMotors.can_send_flag = 0;
}

#ifdef DRIVE_MTR_USE_VESC
/**
 * @brief 向驱动轮电调发送速度命令，将线速度转为电转速
 * 
 * @param vel 单位m/s
 * @note 需要将vel转为电转速 
 */
void SW_DriveMotors_SetSpeed(float vel[4])
{
    if (!RudderChassis.DriveMotors.can_send_flag) // 控制发送周期
        return;
    for (int i = 0; i < 4; i++)
    {
        vel[i] = SW_Speed2eRPM(vel[i]);
        comm_can_set_rpm(RudderChassis.DriveMotors.can_id[i], vel[i]);
        if (1 == i)
            HAL_Delay(1); // 发送间隔1ms
    }
    RudderChassis.DriveMotors.can_send_flag = 0;
}

void SW_DriveMotors_SetCurr(float current[4])
{
    if (!RudderChassis.DriveMotors.can_send_flag) // 控制发送周期
        return;
    for (int i = 0; i < 4; i++)
    {
        comm_can_set_current(RudderChassis.DriveMotors.can_id[i], current[i]);
        if (2 == i)
            HAL_Delay(1); // 发送间隔1ms
    }
    RudderChassis.DriveMotors.can_send_flag = 0;
}

void SW_DriveMotors_SetDuty(float duty[4])
{
    if (!RudderChassis.DriveMotors.can_send_flag) // 控制发送周期
        return;
    for (int i = 0; i < 4; i++)
    {
        comm_can_set_duty(RudderChassis.DriveMotors.can_id[i], duty[i]);
        if (2 == i)
            HAL_Delay(1); // 发送间隔1ms
    }
    RudderChassis.DriveMotors.can_send_flag = 0;
}

/**
 * @brief 手刹模式
 * 
 * @param hand_brake_current 手刹电流
 */
void SW_DriveMotors_Handbrake(float hand_brake_current)
{
    if (!RudderChassis.DriveMotors.can_send_flag) // 控制发送周期
        return;
    for (int i = 0; i < 4; i++)
    {
        comm_can_set_handbrake(RudderChassis.DriveMotors.can_id[i], hand_brake_current);
        if (2 == i)
            HAL_Delay(1); // 发送间隔1ms
    }
}
#endif

#ifdef DRIVE_MTR_USE_ODRIVE
/**
 * @brief 使用ODrive驱动器时的速度设定函数
 * 
 * @param vel 
 */
void SW_DriveMotors_SetSpeed(float vel[4])
{
    if (!RudderChassis.DriveMotors.can_send_flag) // 控制发送频率
        return;
    for (int i = 0; i < 4; i++)
    {
        vel[i] = SW_Speed2eRPM(vel[i]); // >>>请根据不同电机进行换算<<<
        if (2 == i)
            HAL_Delay(1); // 发送间隔1ms
    }
    odrive[0].axis0.mtr_id = RudderChassis.DriveMotors.can_id[0];
    odrive[0].axis0.mtr_id = RudderChassis.DriveMotors.can_id[1];
    odrive[1].axis1.mtr_id = RudderChassis.DriveMotors.can_id[2];
    odrive[1].axis1.mtr_id = RudderChassis.DriveMotors.can_id[3];
    odrive[0].axis0.set_state.input_vel = RudderChassis.DriveMotors.target_rpm[0];
    odrive[0].axis0.set_state.input_vel = RudderChassis.DriveMotors.target_rpm[1];
    odrive[1].axis1.set_state.input_vel = RudderChassis.DriveMotors.target_rpm[2];
    odrive[1].axis1.set_state.input_vel = RudderChassis.DriveMotors.target_rpm[3];
    ODrive_CANSendMsg(&hcan1, &odrive[0], AXIS_0, ODRIVE_MSG_SET_INPUT_VEL);
    ODrive_CANSendMsg(&hcan1, &odrive[0], AXIS_1, ODRIVE_MSG_SET_INPUT_VEL);
    HAL_Delay(1); // 发两个顿一下，不然CAN发送邮箱会炸
    ODrive_CANSendMsg(&hcan1, &odrive[1], AXIS_0, ODRIVE_MSG_SET_INPUT_VEL);
    ODrive_CANSendMsg(&hcan1, &odrive[1], AXIS_1, ODRIVE_MSG_SET_INPUT_VEL);
    RudderChassis.DriveMotors.can_send_flag = 0;
}
#endif

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
        __LIMIT_FROM_TO(pos[i], STEER_WHEEL_MIN_POS, STEER_WHEEL_MAX_POS);
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
        DJIBoard_ReadActualPos(CAN1, i + 1);
    }
}

extern RM_MotorStatus_t RM_MotorStatus[4];
uint8_t SW_PrintMotorStatus_Flag = 0;
void SW_PrintMotorStatus()
{
    if (!(SW_PrintMotorStatus_Flag && TimeFlag_20ms))
        return;
    uprintf("--DJIMotor|");

    uprintf("pos>");
    for (int i = 0; i < 4; i++)
    {
        //除以减速比得到舵轮的实际位置
        uprintf("[%d]%.2f ", i, __RAD2ANGLE(RM_MotorStatus[i].pos / STEER_WHEEL_REDUCTION_RATIO));
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
        // if (speed[i] < 0) // 舵向角度差[90,270]反转的情况
        // {
        //     speed[i] *= -1;
        //     LIMIT(speed[i], DRIVE_WHEEL_MAX_SPEED);
        //     speed[i] *= -1;
        //     continue;
        // }
        __LIMIT(speed[i], DRIVE_WHEEL_MAX_SPEED);
    }
}
//=======================================END=============================================
#endif