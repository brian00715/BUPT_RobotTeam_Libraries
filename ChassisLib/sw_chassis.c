/*******************************************************************
 * @brief 八电机舵轮底盘运动学
 * @author Simon
 * @date 2020/11/1
 * @note 左上右上左下右下编号依次定义为1234，底盘y轴正方向定义为顺着电源开关'-'的方向
 *       x轴正方向定义为绿色12V开关侧
 ******************************************************************/

#ifdef USE_RUDDER_CHASSIS

#include "sw_chassis.h"
#include "stdlib.h"

//========================================private========================================
// 加入自转时速度分量的叠加量应乘以的系数{kx,ky}，对应轮子编号.
static const int8_t prvOmegaRatio[4][2] = {{-1, -1}, {-1, 1}, {1, -1}, {1, 1}};
// 子结构体实体，对外只暴露指针
struct SW_PostureStatus_t prvPostureStatus;
struct SW_TrackStatus_t prvTrackStatus = {0, 0, 0};
struct SW_DriveMotor_t prvDriverMotors;
struct SW_SteerMotor_t prvSteerMotors;
//==========================================END=============================================

//========================================public========================================
struct SW_Chassis_t SW_Chassis; // 底盘全局结构体

float SW_CMD_ChassisSpeed = 0;
float CMD_ChassisDir = 0; //degree
float CMD_ChassisOmega = 0;
PID_t NormalCorrPID = {0.9, 1.5, 0.005, 0, 0, 10, 0, 1.0}; //法向修正向量PD控制
/* 轨迹跟踪时最优PID参数 
   单独调试时最优PID参数0.9, 1.5, 0.005, 0, 0, 10, 0, 1.0*/
PID_t YawPID = {0.1, 0.0, 0.0, 0, 0, 10, 0, 5.0}; //偏航角控制，ω=f(Δθ)
/* 轨迹跟踪时最优YawPID参数0.75, 1.5, 0.0472, 0, 0, 10, 0, 5.0
   单独调试时最优YawPID参数1.8, 2.5, 0.0472, 0, 0, 10, 0, 4.5*/
//==========================================END=============================================

/**
 * @brief 总初始化，包含各指针的关联
 */
void SW_Init()
{
    // 关联结构体指针
    SW_Chassis.DriveMotors = &prvDriverMotors;
    SW_Chassis.SteerMotors = &prvSteerMotors;
    SW_Chassis.PostureStatus = &prvPostureStatus;
    SW_Chassis.TrackStatus = &prvTrackStatus;

    // 关联函数指针
    SW_Chassis.fChassisMove = SW_ChassisMove;
    SW_Chassis.fWheelsControl = SW_SingleCtrl;

    //子初始化
    SW_MotorInit(SW_Chassis.DriveMotors, SW_Chassis.SteerMotors);
    SW_PostureStatusInit();
    SW_Chassis.ctrl_mode = SW_MODE_NONE;
    SW_Chassis.pos_mode = SW_RELATIVE;
}

/** @brief  底盘位姿状态初始化*/
void SW_PostureStatusInit()
{
    SW_Chassis.PostureStatus->omega = 0;
    SW_Chassis.PostureStatus->speed = 0;
    SW_Chassis.PostureStatus->x = 0;
    SW_Chassis.PostureStatus->y = 0;
    SW_Chassis.PostureStatus->yaw = 0;
    SW_Chassis.PostureStatus->speed_x = 0;
    SW_Chassis.PostureStatus->speed_y = 0;
    SW_Chassis.PostureStatus->last_x = 0;
    SW_Chassis.PostureStatus->last_y = 0;
    SW_Chassis.PostureStatus->last_yaw = 0;
    SW_Chassis.target_dir = 0;
    SW_Chassis.target_speed = 0;
    SW_Chassis.target_omega = 0;
}

void SW_TrackStatusInit()
{
    SW_Chassis.TrackStatus->path_index = -1;
    SW_Chassis.TrackStatus->point_index = -1;
    SW_Chassis.TrackStatus->points_set_index = -1;
}

/** 
  * @brief 更新底盘位姿状态
  * @note  在cmd.c的usart_exc_DMA_vega()函数使用DMA进行传输
  */
void SW_UpdatePostureStatus()
{
    // 全场定位can的发送时间间隔为5ms，因此用坐标差除以0.005就是瞬时速度
    SW_Chassis.PostureStatus->speed_x = (SW_Chassis.PostureStatus->x - SW_Chassis.PostureStatus->last_x) / 0.005;                  // m/s
    SW_Chassis.PostureStatus->speed_y = (SW_Chassis.PostureStatus->y - SW_Chassis.PostureStatus->last_y) / 0.005;                  // m/s
    SW_Chassis.PostureStatus->omega = (SW_Chassis.PostureStatus->yaw - SW_Chassis.PostureStatus->last_yaw) / 0.005;                // 弧度/s
    SW_Chassis.PostureStatus->speed = sqrt(pow(SW_Chassis.PostureStatus->speed_x, 2) + pow(SW_Chassis.PostureStatus->speed_y, 2)); // 合成速度

    SW_Chassis.PostureStatus->last_x = SW_Chassis.PostureStatus->x;
    SW_Chassis.PostureStatus->last_y = SW_Chassis.PostureStatus->y;
    SW_Chassis.PostureStatus->last_yaw = SW_Chassis.PostureStatus->yaw;
}

int PrintChassisStatus_Flag = 0; // 手柄或CMD控制开启
void SW_PrintChassisStatus()
{
    if (!(PrintChassisStatus_Flag && TimeFlag_100ms))
        return;
    extern struct SW_PostureStatus_t BUPT_Location1;
    extern struct SW_PostureStatus_t BUPT_Location2;
    // uprintf("--ChassisStatus|x:%7.3f y:%7.3f yaw:%5.2f vx:%5.2f vy:%5.2f \r\n",
    //         SW_Chassis.PostureStatus->x, SW_Chassis.PostureStatus->y,
    //         SW_Chassis.PostureStatus->yaw,
    //         SW_Chassis.PostureStatus->speed_x, SW_Chassis.PostureStatus->speed_y);
    uprintf("-- x:%7.3f y:%7.3f yaw:%5.2f||x:%7.3f y:%7.3f yaw:%5.2f||x:%7.3f y:%7.3f yaw:%5.2f \r\n",
            SW_Chassis.PostureStatus->x, SW_Chassis.PostureStatus->y, RAD2ANGLE(SW_Chassis.PostureStatus->yaw),
            BUPT_Location1.x, BUPT_Location1.y, BUPT_Location1.yaw,
            BUPT_Location2.x, BUPT_Location2.y, BUPT_Location2.yaw);
}

int PrintTargetStatus_Flag = 0;
void SW_PrintTargetStatus()
{
    if (!(PrintTargetStatus_Flag && TimeFlag_20ms))
        return;
    uprintf("--target|speed:%.3f dir:%.3f omega:%.3f\r\n",
            SW_Chassis.target_speed, SW_Chassis.target_dir, SW_Chassis.target_omega);
}

extern float CMD_TargetYaw;
int SW_PointSetIndex = 0; // 点集下标
/**
  * @brief 舵轮底盘执行函数
  * @note 在main函数中每5ms执行一次
  */
void SW_EXE()
{
    // 不同的控制模式决定target更新的方式
    switch (SW_Chassis.ctrl_mode)
    {
    case SW_MODE_NONE:
        break;
    case SW_HANDLE:
        SW_Handle_EXE();
        SW_Chassis.fChassisMove(SW_Chassis.target_speed, SW_Chassis.target_dir, SW_Chassis.target_omega); // 底层执行函数
        break;
    case SW_CMD:
        SW_Chassis.target_speed = SW_CMD_ChassisSpeed;
        SW_Chassis.target_dir = ANGLE2RAD(CMD_ChassisDir);
        SW_Chassis.target_omega = CMD_ChassisOmega;
        SW_Chassis.fChassisMove(SW_Chassis.target_speed, SW_Chassis.target_dir, SW_Chassis.target_omega); // 底层执行函数
        break;
    case SW_TRACK:
        SW_Chassis.pos_mode = SW_ABSOLUTE;
        SW_TrackPathSets(SW_Chassis.TrackStatus->path_index);
        break;
    case SW_TUNING:
        SW_YawTuning(CMD_TargetYaw);
        SW_Chassis.fChassisMove(SW_Chassis.target_speed, SW_Chassis.target_dir, SW_Chassis.target_omega); // 底层执行函数
        break;
    default:
        break;
    }
}

/**
 * @brief 底盘速度矢量控制，最底层调用
 * @param speed 速度大小/rpm
 * @param dir 速度方向/rad
 * @param omega 自转角速度(rad/s)，逆时针为正方向
 * @note 控制方法为四个轮子同时转向，始终保持一个方向，并根据速度方向偏差转动每个单独轮以进行补偿
 *       规定dir的范围是[0,pi],[-pi,0]
 **/
void SW_ChassisMove(float speed, float dir, float omega)
{
    float absolute_angle_offset = 0; //使用绝对坐标时，根据全场定位测得的偏航角进行补偿
    if (SW_Chassis.pos_mode == SW_ABSOLUTE)
    {
        absolute_angle_offset = SW_Chassis.PostureStatus->yaw - 1.5708;
        if (absolute_angle_offset > PI)
        {
            absolute_angle_offset = -(2 * PI - absolute_angle_offset);
        }
    }
    dir -= absolute_angle_offset;
    float vx = speed * cos(dir); // 速度分量
    float vy = speed * sin(dir);
    LIMIT(omega, MAX_ROTATE_VEL); // omega需要参与运算，故提前限制大小
    float target_pos[4];
    float target_speed[4];
    if (speed != 0 || omega != 0)
    {
        for (int i = 0; i < 4; i++)
        {
            float self_turn_vel_x = omega * SW_Wheel_Front2Back / 2; // 自转切向速度的分量
            float self_turn_vel_y = omega * SW_Wheel_Left2Right / 2;
            float vel_sum_x = vx + prvOmegaRatio[i][0] * self_turn_vel_x; // 速度合成，考虑底盘形状
            float vel_sum_y = vy + prvOmegaRatio[i][1] * self_turn_vel_y;
            float real_vel = sqrt(pow(vel_sum_x, 2) + pow(vel_sum_y, 2));
            float real_dir = atan2f(vel_sum_y, vel_sum_x); // 附加角速度后的真实速度方向
            SW_TurnMinorArc(RM_MotorStatus[i].pos / STEER_WHEEL_REDUCTION_RATIO, &real_dir, &real_vel);
#ifdef DEBUG
            //uprintf("--[%d]target_speed=%.4f target_dir=%d\r\n", i, real_vel, real_dir);
#endif
            target_pos[i] = real_dir * STEER_WHEEL_REDUCTION_RATIO; // 乘舵轮减速比
            target_speed[i] = real_vel;
        }
    }
    else // 速度和角速度都为0时要保证舵向有效
    {
        for (int i = 0; i < 4; i++)
        {
            SW_TurnMinorArc(RM_MotorStatus[i].pos / STEER_WHEEL_REDUCTION_RATIO, &dir, &target_speed[i]);
            target_pos[i] = dir * STEER_WHEEL_REDUCTION_RATIO;
            target_speed[i] = 0;
        }
    }

    // 数值限制
    SW_SteerMotors_LimitPos(target_pos);
    SW_DriveMotors_LimitSpeed(target_speed);

    //向驱动板发送命令
    SW_Chassis.SteerMotors->setPos(target_pos);
    SW_Chassis.DriveMotors->setRPM(target_speed);
}

/**
 * @brief 两帧角度差太大时通过反转驱动轮方向减小转动量，使舵向电机走劣弧
 */
void SW_TurnMinorArc(float now_dir, float *target_dir, float *target_speed)
{
    float delta_dir = *target_dir - now_dir; // 偏差量

    if (fabs(delta_dir) > PI / 2 && fabs(delta_dir) < ANGLE2RAD(270))
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
        *target_dir = now_dir + delta_dir;
    }
    // if (TimeFlag_100ms)
    // {
    //     uprintf("now_dir:%.4f target_dir:%.2f delta_dir:%.2f\r\n", now_dir, *target_dir, delta_dir);
    // }
}



/**
 * @brief 急停，两侧车轮同时向内旋转，利用滑动摩擦刹车
 * @param acc 刹车加速度,相对值
 */
void SW_SuddenStop(float acc)
{
}

extern int YawTuning_Start;
/**
 * @brief 原地差速自转，用于跑完轨迹后的偏航角调整
 * 
 * @param target_yaw 
 * @note 重新对每个轮子的期望方向进行分配，偏差值大于60°则不分配速度，避免计算出
 *       过大的线速度。缺点是会造成滑动摩擦
 */
void SW_Rotate(float target_yaw)
{
    // if (!YawTuning_Start)
    // {
    //     return;
    // }
    // float delta_angle = target_yaw - SW_Chassis.PostureStatus->yaw;
    // delta_angle = AngleLimitPI(delta_angle);
    // target_yaw = SW_Chassis.PostureStatus->yaw + delta_angle;
    // float omega = PID_Release(&YawPID, target_yaw, SW_Chassis.PostureStatus->yaw); // 暂时不加入角速度内环
    // float self_turn_vel = omega * SW_Wheel2ChassisCenter / 2;                      // 自转切向速度

    // float target_speed[4] = {0};
    // const float SW_ShapeAngle = atan(SW_ShapeFactor); // 对角线与x边的夹角
    // float wheel_static_angle[4] = {
    //     -PI / 2 - SW_ShapeAngle,
    //     PI / 2 + SW_ShapeAngle,
    //     -PI / 2 + SW_ShapeAngle,
    //     PI / 2 - SW_ShapeAngle}; // 只有自转时舵轮应有的角度
    // for (int i = 0; i < 4; i++)
    // {
    //     float delta_angle = wheel_static_angle[i] - SW_Chassis.SteerMotors->now_pos[i];
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
    //     if (fabs(delta_angle) < ANGLE2RAD(60)) // 差值大于60°不分配速度
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
    extern struct Handle_t SW_Handle;
    SW_Handle.left_rocker.length = (uint16_t)sqrt(pow(SW_Handle.left_rocker.x, 2) +
                                                  pow(SW_Handle.left_rocker.y, 2));
    SW_Handle.right_rocker.length = (uint16_t)sqrt(pow(SW_Handle.right_rocker.x, 2) +
                                                   pow(SW_Handle.right_rocker.y, 2));
    float target_speed[4]={0};
    if (SW_Handle.right_rocker.length > 108) // 只有摇杆顶到头才开启转动
    {
        // 计算右摇杆相角与与pi/2的偏差值(rad)，大小控制角速度
        double angle_offset = AngleLimitDiff(atan2(SW_Handle.right_rocker.y, SW_Handle.right_rocker.x), PI / 2);
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
        float self_turn_vel = omega * SW_Wheel2ChassisCenter / 2; // 自转切向速度
        target_speed[0] = -self_turn_vel;
        target_speed[1] = -self_turn_vel;
        target_speed[2] = self_turn_vel;
        target_speed[3] = self_turn_vel;
    }

    SW_Chassis.DriveMotors->setRPM(target_speed);
    // ## 舵向不变 ##

    // if (TimeFlag_20ms)
    // {
    //     uprintf("--Tuing|target_yaw:%.2f now_yaw:%.2f omega:%.2f\r\n", target_yaw, SW_Chassis.PostureStatus->yaw, omega);
    // }
}

/**
 * @brief 单轮控制
 * @param index 电机编号
 * @param speed 驱动电机转速/rpm
 * @param dir 速度方向/rad
 */
void SW_SingleCtrl(int index, float speed, float dir)
{
    // 数值限位
    LIMIT(speed, DRIVE_WHEEL_MAX_SPEED);
    LIMIT(dir, STEER_WHEEL_MAX_POS);
    //向驱动板发送命令
    DJI_PosCtrl(CAN1, index, ABSOLUTE_MODE, (int32_t)RAD2ANGLE(dir * STEER_WHEEL_REDUCTION_RATIO));
    comm_can_set_rpm(SW_Chassis.DriveMotors->id[index], speed);
}

/**
 * @brief 单独控制偏航角
 * @param yaw 目标偏航角/rad
 * @param max_omega 最大角速度/rpm
 * @note 原地自转，不能在运动过程中调用
 */
void SW_ChassisCtrlYaw(float yaw, uint32_t max_omega)
{
    float pos[4];
    if (max_omega < STEER_WHEEL_MAX_SPEED)
    {
        max_omega = STEER_WHEEL_MAX_SPEED;
    }
    for (int i = 0; i < 4; i++) // 配置最大转度
    {
        PosLoopCfg(CAN1, i + 1, 0, 0, max_omega);
        pos[i] = yaw;
    }
    SW_Chassis.SteerMotors->setPos(pos);
}



int YawTuning_Start = 0;
/**
 * @brief 调试偏航角PID
 * 
 * @param target_yaw 
 */
void SW_YawTuning(float target_yaw)
{
    if (!YawTuning_Start)
    {
        SW_Chassis.target_speed = 0;
        SW_Chassis.target_omega = 0;
        return;
    }
    float now_yaw = SW_Chassis.PostureStatus->yaw;
    float delta_angle = target_yaw - now_yaw;
    delta_angle = AngleLimitPI(delta_angle);
    target_yaw = now_yaw + delta_angle;
    float omega = PID_Release(&YawPID, target_yaw, now_yaw);

    SW_Chassis.target_omega = omega;
    SW_Chassis.target_speed = 0;
    if (TimeFlag_20ms)
    {
        uprintf("--Tuing|target_yaw:%.2f now_yaw:%.2f omega:%.2f\r\n", target_yaw, now_yaw, omega);
    }
}
//==========================================END=============================================
#endif