/*******************************************************************
 * @brief 舵轮底盘运动学
 * @author Simon
 * @date 2021/04/15
 * @note 顺时针编号依次定义为1234，底盘y轴正方向定义为顺着电源开关'-'的方向
 *       x轴正方向定义为绿色12V开关侧
 ******************************************************************/
#include "rudder_chassis.h"
#include "motor_driver.h"
#include "vesc_can.h"

#ifdef USE_CHASSIS_RUDDER
#include <stdlib.h>

//========================================private========================================
//==========================================END=============================================

//========================================public========================================
RudderChassis_s RudderChassis; // 底盘全局结构体

extern PID_s NormalCorrPID_x, NormalCorrPID_y; //法向修正向量PD控制
extern PID_s YawPID;                           //偏航角控制，ω=f(Δθ)
//==========================================END=============================================

/**
 * @brief 总初始化，包含各指针的关联
 */
void RudderChassis_Init()
{
    BaseChassis_Init();
    SW_MotorInit(&RudderChassis.DriveMotors, &RudderChassis.SteerMotors);
    RudderChassis.base = &BaseChassis;

    // 关联函数指针
    RudderChassis.base->fChassisMove = RudderChassis_Move;

    RudderChassis.base->ctrl_mode = CTRL_MODE_NONE;
    RudderChassis.base->pos_mode = POS_MODE_RELATIVE;
    NormalCorrPID_x.Kp = 0.9;
    NormalCorrPID_x.Ki = 0.00;
    NormalCorrPID_x.Kd = 1.5;
    NormalCorrPID_x.int_max = 10;
    NormalCorrPID_x.int_duty = 1.0;
    NormalCorrPID_y.Kp = 0.9;
    NormalCorrPID_y.Ki = 0.005;
    NormalCorrPID_y.Kd = 1.5;
    NormalCorrPID_y.int_max = 10;
    NormalCorrPID_y.int_duty = 1.0;
    YawPID.Kp = 3.5;
    YawPID.Ki = 0.01;
    YawPID.Kd = 0.0;
    YawPID.int_max = 10;
    YawPID.int_duty = 10.0;
}

extern float CMD_TargetYaw;
int RudderPointSetIndex = 0; // 点集下标
/**
  * @brief 舵轮底盘执行函数
  * @note 在main函数中每5ms执行一次
  */
void RudderChassis_Exe()
{
    Chassis_UpdatePostureStatus();

    switch (RudderChassis.base->ctrl_mode)
    {
    case CTRL_MODE_NONE:
        RudderChassis.base->target_speed = 0;
        RudderChassis.base->target_omega = 0;
        break;
    case CTRL_MODE_HANDLE:
        Handle_Exe();
        break;
    case CTRL_MODE_CMD:
        RudderChassis.base->pos_mode = POS_MODE_RELATIVE;
        RudderChassis.base->target_speed = CMD_TargetSpeed;
        RudderChassis.base->target_dir = CMD_TargetDir;
        RudderChassis.base->target_omega = CMD_TargetOmega;
        break;
    case CTRL_MODE_TRACK:
        RudderChassis.base->pos_mode = POS_MODE_ABSOLUTE;
        Chassis_TrackPathSets(RudderChassis.base->TrackStatus.track_path_index);
        break;
    case CTRL_MODE_GO_TO_POINT:
        RudderChassis.base->pos_mode = POS_MODE_ABSOLUTE;
        Chassis_Go2Point(RudderChassis.base->Go2PointStatus.target_point,
                         RudderChassis.base->Go2PointStatus.target_yaw,
                         RudderChassis.base->Go2PointStatus.start_speed,
                         RudderChassis.base->Go2PointStatus.final_speed);

        break;
    case CTRL_MODE_TUNING:
        Chassis_YawTuning(CMD_TargetYaw);
        break;
    case CTRL_MODE_EXTERNAL:
        break;
    default:
        break;
    }
    RudderChassis_Move(RudderChassis.base->target_speed, RudderChassis.base->target_dir, RudderChassis.base->target_omega);
}

// 加入自转时速度分量的叠加量应乘以的系数{kx,ky}，对应轮子编号.
static const int8_t omega_ratio[4][2] = {{-1, -1}, {-1, 1}, {1, 1}, {1, -1}};
/**
 * @brief 底盘速度矢量映射为四个轮子的线速度与舵向角
 * @param speed 速度大小/rpm 尽量传入非负数
 * @param dir 速度方向/rad
 * @param omega 自转角速度(rad/s)，逆时针为正方向
 * @note 控制方法为四个轮子同时转向，始终保持一个方向，并根据速度方向偏差转动每个单独轮以进行补偿
 *       规定dir的范围是[0,pi],[-pi,0]
 **/
void RudderChassis_Move(float speed, float dir, float omega)
{
    float absolute_angle_offset = 0; //使用绝对坐标时，根据全场定位测得的偏航角进行补偿
    if (RudderChassis.base->pos_mode == POS_MODE_ABSOLUTE)
    {
        absolute_angle_offset = RudderChassis.base->PostureStatus.yaw - 1.5708;
        if (absolute_angle_offset > PI)
        {
            absolute_angle_offset = -(2 * PI - absolute_angle_offset);
        }
    }
    dir -= absolute_angle_offset;
    float vx = speed * cos(dir); // 速度分量
    float vy = speed * sin(dir);
    __LIMIT(omega, MAX_ROTATE_VEL); // omega需要参与运算，故提前限制大小
    float target_pos[4];
    float target_speed[4];
    if (speed != 0 || omega != 0)
    {
        for (int i = 0; i < 4; i++)
        {
            float self_turn_vel_x = omega * RudderChassis_Front2Back / 2; // 自转切向速度的分量
            float self_turn_vel_y = omega * RudderChassis_Left2Right / 2;
            float vel_sum_x = vx + omega_ratio[i][0] * self_turn_vel_x; // 速度合成，考虑底盘形状
            float vel_sum_y = vy + omega_ratio[i][1] * self_turn_vel_y;
            float real_vel = sqrt(pow(vel_sum_x, 2) + pow(vel_sum_y, 2));
            float real_dir = atan2f(vel_sum_y, vel_sum_x); // 附加角速度后的真实速度方向
            RudderChassis_TurnMinorArc(RM_MotorStatus[i].pos / STEER_WHEEL_REDUCTION_RATIO, &real_dir, &real_vel);
#ifdef DEBUG
            //uprintf("--[%d]target_speed=%.4f target_dir=%d\r\n", i, real_vel, real_dir);
#endif
            target_pos[i] = real_dir * STEER_WHEEL_REDUCTION_RATIO;                                        // 乘舵轮减速比
            real_vel = real_vel > 0 ? real_vel + DRIVE_WHEEL_MIN_SPEED : real_vel - DRIVE_WHEEL_MIN_SPEED; // 补偿启动速度
            target_speed[i] = real_vel;
        }
    }
    else // 速度和角速度都为0时要保证舵向有效
    {
        for (int i = 0; i < 4; i++)
        {
            RudderChassis_TurnMinorArc(RM_MotorStatus[i].pos / STEER_WHEEL_REDUCTION_RATIO, &dir, &target_speed[i]);
            target_pos[i] = dir * STEER_WHEEL_REDUCTION_RATIO;
            target_speed[i] = 0;
        }
    }

    // 数值限制
    SW_SteerMotors_LimitPos(target_pos);
    SW_DriveMotors_LimitSpeed(target_speed);

    //向驱动板发送命令

    RudderChassis.SteerMotors.fSetPos(target_pos);
    if (RudderChassis.base->handbrake_flag)
    {
        RudderChassis.DriveMotors.fHandbrake(10.0);
        uprintf("RudderChassis|sended handbrake command!\r\n");
    }
    else
    {
        RudderChassis.DriveMotors.fSetSpeed(target_speed);
    }
}

/**
 * @brief 两帧角度差太大时通过反转驱动轮方向减小转动量，使舵向电机走劣弧
 */
void RudderChassis_TurnMinorArc(float now_dir, float *target_dir, float *target_speed)
{
    float delta_dir = *target_dir - now_dir; // 偏差量

    if (fabs(delta_dir) > __ANGLE2RAD(90) && fabs(delta_dir) < __ANGLE2RAD(270))
    {
        if (*target_dir < 0)
        {
            *target_dir += PI;
        }
        else
        {
            *target_dir -= PI;
        }
        *target_speed = -*target_speed;
    }
    else
    {
        if (fabs(delta_dir) > __ANGLE2RAD(270))
        {
            //  uprintf(">270,delta_dir:%.2f\r\n",delta_dir);
            if (delta_dir > __ANGLE2RAD(270))
            {
                delta_dir -= 6.28;
            }
            else if (delta_dir < -__ANGLE2RAD(270))
            {
                delta_dir += 6.28;
            }
        }
        *target_dir = now_dir + delta_dir;
    }
    if (TimeFlag_100ms)
    {
        //uprintf("now_dir:%.4f target_dir:%.2f delta_dir:%.2f\r\n", now_dir, *target_dir, delta_dir);
    }
}

/**
 * @brief 急停
 * @param acc 刹车加速度,相对值
 */
void RudderChassis_SuddenStop(float acc)
{
}

/**
 * @brief 原地差速自转，用于跑完轨迹后的偏航角调整
 * 
 * @param target_yaw 
 * @note 重新对每个轮子的期望方向进行分配，偏差值大于60°则不分配速度，避免计算出
 *       过大的线速度。缺点是会造成滑动摩擦
 */
void RudderChassis_Rotate(float target_yaw)
{
    // if (!YawTuning_Start)
    // {
    //     return;
    // }
    // float delta_angle = target_yaw - RudderChassis.PostureStatus->yaw;
    // delta_angle = AngleLimitPI(delta_angle);
    // target_yaw = RudderChassis.PostureStatus->yaw + delta_angle;
    // float omega = PID_GetOutput(&YawPID, target_yaw, RudderChassis.PostureStatus->yaw); // 暂时不加入角速度内环
    // float self_turn_vel = omega * RudderWheel2ChassisCenter / 2;                      // 自转切向速度

    // float target_speed[4] = {0};
    // const float RudderShapeAngle = atan(RudderShapeFactor); // 对角线与x边的夹角
    // float wheel_static_angle[4] = {
    //     -PI / 2 - RudderShapeAngle,
    //     PI / 2 + RudderShapeAngle,
    //     -PI / 2 + RudderShapeAngle,
    //     PI / 2 - RudderShapeAngle}; // 只有自转时舵轮应有的角度
    // for (int i = 0; i < 4; i++)
    // {
    //     float delta_angle = wheel_static_angle[i] - RudderChassis.SteerMotors->now_pos[i];
    //     delta_angle = AngleLimitPI(delta_angle); // 限制角度在[0,pi]
    //     char reverse_flag = 0;
    //     if (fabs(delta_angle) > PI / 2) // 限制绝对值在[0,pi/2]
    //     {
    //         if (delta_angle > 0)
    //         {
    //             delta_angle = -PI + delta_angle;
    //         }
    //         else
    //         {
    //             delta_angle = PI + delta_angle;
    //         }
    //         reverse_flag = 1;
    //     }
    //     if (fabs(delta_angle) < __ANGLE2RAD(60)) // 差值大于60°不分配速度
    //     {
    //         target_speed[i] = fabs(self_turn_vel / cos(delta_angle));
    //         if (reverse_flag)
    //         {
    //             target_speed[i] = -target_speed[i];
    //         }
    //     }
    //     else
    //     {
    //         target_speed[i] = 0;
    //     }
    // }
    extern struct Handle_t Handle;
    Handle.left_rocker.length = (uint16_t)sqrt(pow(Handle.left_rocker.x, 2) +
                                               pow(Handle.left_rocker.y, 2));
    Handle.right_rocker.length = (uint16_t)sqrt(pow(Handle.right_rocker.x, 2) +
                                                pow(Handle.right_rocker.y, 2));
    float target_speed[4] = {0};
    if (Handle.right_rocker.length > 108) // 只有摇杆顶到头才开启转动
    {
        // 计算右摇杆相角与与pi/2的偏差值(rad)，大小控制角速度
        double angle_offset = AngleLimitDiff(atan2(Handle.right_rocker.y, Handle.right_rocker.x), PI / 2);
        if (fabs(angle_offset) < 0.314)
        {
            angle_offset = 0;
        }
        else if (angle_offset > 0)
        {
            angle_offset -= 0.314;
        }
        else if (angle_offset < 0)
        {
            angle_offset += 0.314;
        }
        float omega = 6 * angle_offset;
        float self_turn_vel = omega * RudderChassis_Wheel2ChassisCenter / 2; // 自转切向速度
        target_speed[0] = -self_turn_vel;
        target_speed[1] = -self_turn_vel;
        target_speed[2] = self_turn_vel;
        target_speed[3] = self_turn_vel;
    }

    RudderChassis.DriveMotors.fSetSpeed(target_speed);
    // ## 舵向不变 ##

    // if (TimeFlag_20ms)
    // {
    //     uprintf("--Tuing|target_yaw:%.2f now_yaw:%.2f omega:%.2f\r\n", target_yaw, RudderChassis.PostureStatus->yaw, omega);
    // }
}

/**
 * @brief 单轮控制
 * @param index 电机编号
 * @param speed 驱动电机转速/rpm
 * @param dir 速度方向/rad
 */
void RudderChassis_SingleCtrl(int index, float speed, float dir)
{
    // 数值限位
    __LIMIT(speed, DRIVE_WHEEL_MAX_SPEED);
    __LIMIT(dir, STEER_WHEEL_MAX_POS);
    //向驱动板发送命令
    DJIBoard_PosCtrl(CAN1, index, ABSOLUTE_MODE, (int32_t)__RAD2ANGLE(dir * STEER_WHEEL_REDUCTION_RATIO));
    comm_can_set_rpm(RudderChassis.DriveMotors.can_id[index], speed);
}

/**
 * @brief 单独控制偏航角
 * @param yaw 目标偏航角/rad
 * @param max_omega 最大角速度/rpm
 * @note 原地自转，不能在运动过程中调用
 */
void RudderChassis_CtrlYaw(float yaw, uint32_t max_omega)
{
    float pos[4];
    if (max_omega < STEER_WHEEL_MAX_SPEED)
    {
        max_omega = STEER_WHEEL_MAX_SPEED;
    }
    for (int i = 0; i < 4; i++) // 配置最大转度
    {
        DJIBoard_PosLoopCfg(CAN1, i + 1, 0, 0, max_omega);
        pos[i] = yaw;
    }
    RudderChassis.SteerMotors.fSetPos(pos);
}

//==========================================END=============================================
#endif