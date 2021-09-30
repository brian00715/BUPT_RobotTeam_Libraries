/**
 * @file base_chassis.c
 * @author simon
 * @brief 底盘基类
 * @version 1.0
 * @date 2021-07-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "base_chassis.h"
#include "chassis_common_config.h"
#include "main.h"
#include "rudder_chassis.h"
#include "omni_chassis.h"
#include "steer_wheel.h"

#define ARRIVED_CIRCIE_TH (0.007) // m 到达半径阈值
#define LOCK_CIRCLE_TH (0.2)      // m 启用原地锁定PID的距离半径阈值

BaseChassis_s BaseChassis; // 基类对象，子类继承时引用其指针
float CMD_TargetSpeed = 0;
float CMD_TargetDir = 0;
float CMD_TargetOmega = 0;
float CMD_TargetYaw = 1.5708;

char YawTuning_Start;

/* PID结构体,请根据所使用的底盘实际调试-----------------------------*/
PID_s NormalCorrPID_x = {0}; //
PID_s NormalCorrPID_y = {0};
PID_s Chassis_SpeedCtrlPID = {0};
PID_s Chassis_OmegaCtrlPID = {0};
PID_s YawPID = {0};
PID_s LockPID = {0}; // 跑点模式锁止PID

static float location_raw_x, location_raw_y,
    location_raw_speed_x, location_raw_speed_y;

/**
 * @brief 底盘基类初始化
 */
void BaseChassis_Init()
{
    //BaseChassis.fChassisMove在子类的初始化函数中关联
    Chassis_PostureStatusInit();
    BaseChassis.fChassisMove = NULL;
    BaseChassis.ctrl_mode = CTRL_MODE_NONE;
    BaseChassis.pos_mode = POS_MODE_RELATIVE;
    BaseChassis.handbrake_flag = 0;
    BaseChassis.lock_yaw_flag = 0;
    BaseChassis.target_yaw = 1.5708;
    BaseChassis.target_omega = 0;
    BaseChassis.target_speed = 0;
    BaseChassis.target_dir = 0;
    BaseChassis.yaw_ctrl_th = 0.01745; // 1°的死区

    BaseChassis.TrackStatus.track_path_index = 0;
    BaseChassis.TrackStatus.point_index = -1;
    BaseChassis.TrackStatus.finished = 0;
    BaseChassis.TrackStatus.start = 0;
    BaseChassis.Go2PointStatus.points_set_index = -1;
    BaseChassis.Go2PointStatus.enable_always_yaw_ctrl = 1;
    BaseChassis.Go2PointStatus.start = 0;
    BaseChassis.Go2PointStatus.enable = 0;
    BaseChassis.Go2PointStatus.arrived = 0;
    BaseChassis.Go2PointStatus.start_speed = 0;
    BaseChassis.Go2PointStatus.final_speed = 0;
    BaseChassis.Go2PointStatus.disable_yaw_ctrl = 0;
    BaseChassis.Go2PointStatus.target_yaw = 1.5708;
    BaseChassis.Go2PointStatus.target_point.x = 0;
    BaseChassis.Go2PointStatus.target_point.y = 0;
    BaseChassis.Go2PointStatus.start_point.x = 0;
    BaseChassis.Go2PointStatus.start_point.y = 0;
    BaseChassis.Go2PointStatus.min_speed = 0;
    BaseChassis.Go2PointStatus.max_speed = DRIVE_WHEEL_MAX_SPEED;

    NormalCorrPID_x.Kp = 0.2;
    NormalCorrPID_x.Ki = 0;
    NormalCorrPID_x.Kd = 0.000;
    NormalCorrPID_x.int_max = 10;
    NormalCorrPID_x.int_duty = 1.0;
    NormalCorrPID_x.ctrl_max = 10;

    NormalCorrPID_y.Kp = 0.2;
    NormalCorrPID_y.Ki = 0;
    NormalCorrPID_y.Kd = 0.000;
    NormalCorrPID_y.int_max = 10;
    NormalCorrPID_y.int_duty = 1.0;
    NormalCorrPID_y.ctrl_max = 10;

    YawPID.Kp = 5.00;
    YawPID.Ki = 0.00;
    YawPID.Kd = 0.0;
    YawPID.int_max = 10;
    YawPID.int_duty = 10.0;
    YawPID.ctrl_max = 5.0;

    LockPID.Kp = 0.7;
    LockPID.Ki = 0.00;
    LockPID.Kd = 0.35;
    LockPID.int_max = 1.0;
    LockPID.int_duty = 5.0;
    LockPID.ctrl_max = 1.0;

    Chassis_SpeedCtrlPID.Kp = 1.5;
    Chassis_SpeedCtrlPID.Ki = 0.01;
    Chassis_SpeedCtrlPID.Kd = 0;
    Chassis_SpeedCtrlPID.ctrl_max = 5.0;
    Chassis_SpeedCtrlPID.int_max = 10.0;

    Chassis_OmegaCtrlPID.Kp = 10;
    Chassis_OmegaCtrlPID.Ki = 0;
    Chassis_OmegaCtrlPID.Kd = 0.5;
    Chassis_OmegaCtrlPID.ctrl_max = 10.0;
    Chassis_OmegaCtrlPID.int_duty = 1;
    Chassis_OmegaCtrlPID.int_max = 10.0;
}

/** @brief  底盘位姿状态初始化*/
void Chassis_PostureStatusInit()
{
    BaseChassis.PostureStatus.omega = 0;
    BaseChassis.PostureStatus.speed = 0;
    BaseChassis.PostureStatus.x = 0;
    BaseChassis.PostureStatus.y = 0;
    BaseChassis.PostureStatus.yaw = 0;
    BaseChassis.PostureStatus.speed_x = 0;
    BaseChassis.PostureStatus.speed_y = 0;
    BaseChassis.PostureStatus.last_x = 0;
    BaseChassis.PostureStatus.last_y = 0;
    BaseChassis.PostureStatus.last_yaw = 0;
    BaseChassis.PostureStatus.pos_corr_x = 0;
    BaseChassis.PostureStatus.pos_corr_y = 0;
    BaseChassis.target_dir = 0;
    BaseChassis.target_speed = 0;
    BaseChassis.target_omega = 0;
}

/** 
  * @brief 由用户实现 | 更新底盘位姿状态
  * @note  此函数调用频率不应低于里程计坐标发布频率
  */
void Chassis_UpdatePostureStatus()
{
    // >>>以下为东大全场定位使用<<<
    // 全场定位can的发送时间间隔为5ms，因此用坐标差除以0.005就是瞬时速度
    // BaseChassis.PostureStatus.speed_x = (BaseChassis.PostureStatus.x - BaseChassis.PostureStatus.last_x) / 0.005;                  // m/s
    // BaseChassis.PostureStatus.speed_y = (BaseChassis.PostureStatus.y - BaseChassis.PostureStatus.last_y) / 0.005;                  // m/s
    // BaseChassis.PostureStatus.omega = (BaseChassis.PostureStatus.yaw - BaseChassis.PostureStatus.last_yaw) / 0.005;                // 弧度/s
    // BaseChassis.PostureStatus.speed = sqrt(pow(BaseChassis.PostureStatus.speed_x, 2) + pow(BaseChassis.PostureStatus.speed_y, 2)); // 合成速度
    // BaseChassis.PostureStatus.last_x = BaseChassis.PostureStatus.x;
    // BaseChassis.PostureStatus.last_y = BaseChassis.PostureStatus.y;
    // BaseChassis.PostureStatus.last_yaw = BaseChassis.PostureStatus.yaw;

    // >>>以下为BUPT全场定位使用<<< （请根据实际安装方式换算）
    BaseChassis.PostureStatus.x =
        -location_raw_x * cos(__ANGLE2RAD(45)) -
        location_raw_y * cos(__ANGLE2RAD(45));
    BaseChassis.PostureStatus.y =
        location_raw_x * cos(__ANGLE2RAD(45)) -
        location_raw_y * cos(__ANGLE2RAD(45));

    float tmp = BaseChassis.PostureStatus.y;
    BaseChassis.PostureStatus.y = BaseChassis.PostureStatus.x + BaseChassis.PostureStatus.pos_corr_y;
    BaseChassis.PostureStatus.x = -tmp + BaseChassis.PostureStatus.pos_corr_x;

    BaseChassis.PostureStatus.speed_x =
        -location_raw_speed_x * cos(__ANGLE2RAD(45)) -
        location_raw_speed_y * cos(__ANGLE2RAD(45));
    BaseChassis.PostureStatus.speed_y =
        -location_raw_speed_x * cos(__ANGLE2RAD(45)) +
        location_raw_speed_y * cos(__ANGLE2RAD(45));
    tmp = BaseChassis.PostureStatus.speed_y;
    BaseChassis.PostureStatus.speed_y = BaseChassis.PostureStatus.speed_x;
    BaseChassis.PostureStatus.speed_x = tmp;
}

/**
 * @brief 对底盘进行宏观闭环速度控制，通过轮式里程计测速.以一定频率执行
 * 
 * @param target_speed 计算得到的输入速度大小
 * @param target_dir 
 * @param target_omega 
 * @return 实际期望值，传递给fChassisMove
 */
void Chassis_MotionCtrl(void)
{
    float now_speed = sqrt(BaseChassis.PostureStatus.speed_x * BaseChassis.PostureStatus.speed_x +
                           BaseChassis.PostureStatus.speed_y * BaseChassis.PostureStatus.speed_y);
    float now_omega = BaseChassis.PostureStatus.omega;
    BaseChassis.speed_ctrl = PID_GetOutput(&Chassis_SpeedCtrlPID, BaseChassis.target_speed, now_speed);
    BaseChassis.omega_ctrl = PID_GetOutput(&Chassis_OmegaCtrlPID, BaseChassis.target_omega, now_omega);
    if (fabs(BaseChassis.target_omega) < 0.1) // 死区
    {
        BaseChassis.omega_ctrl = 0;
    }
    if (BaseChassis.fChassisMove != NULL)
    {
        BaseChassis.fChassisMove(0, BaseChassis.target_dir, BaseChassis.omega_ctrl);
    }
}

Point2D_s CMD_Chassis_TargetPoint = {0};
float CMD_Chassis_TargetYaw = 0;
/**
 * @brief 跑点集,使用跑单个点的方法
 * @param index 点集序号
 */
void Chassis_TrackPoints(int index)
{
    switch (index)
    {
    case 0:
        BaseChassis.fChassisMove(0, 90, 0);
        break;
    case 1:
    {
        CMD_Chassis_TargetPoint.x = 0;
        CMD_Chassis_TargetPoint.y = 0;
        Chassis_Go2Point(CMD_Chassis_TargetPoint, 1.5708, 0.2, 0);
        break;
    }
    case 2:
        CMD_Chassis_TargetPoint.x = 0.5;
        CMD_Chassis_TargetPoint.y = 0.5;
        Chassis_Go2Point(CMD_Chassis_TargetPoint, 1.5708, 0.2, 0);
        break;
    case 3:
        CMD_Chassis_TargetPoint.x = -0.5;
        CMD_Chassis_TargetPoint.y = 0.5;
        Chassis_Go2Point(CMD_Chassis_TargetPoint, 1.5708, 0.2, 0);
        break;
    case 4:
        CMD_Chassis_TargetPoint.x = -0.5;
        CMD_Chassis_TargetPoint.y = -0.5;
        Chassis_Go2Point(CMD_Chassis_TargetPoint, 1.5708, 0.2, 0);
        break;
    default:
        break;
    }
}

void Chassis_Go2Point(Point2D_s target, float target_yaw, float start_spd, float end_spd)
{
    if (!BaseChassis.Go2PointStatus.enable)
    {
        return;
    }

    if (BaseChassis.Go2PointStatus.start == 1) // 保存起点
    {
        BaseChassis.Go2PointStatus.start = 0;
        BaseChassis.Go2PointStatus.start_point.x = BaseChassis.PostureStatus.x;
        BaseChassis.Go2PointStatus.start_point.y = BaseChassis.PostureStatus.y;
        BaseChassis.Go2PointStatus.arrived = 0;
    }

    float distance = sqrt(pow(BaseChassis.PostureStatus.x - target.x, 2) +
                          pow(BaseChassis.PostureStatus.y - target.y, 2));

    // 换算偏差量限制到[0,pi]
    float now_yaw = BaseChassis.PostureStatus.yaw;
    float delta_angle = target_yaw - now_yaw;
    delta_angle = AngleLimitPI(delta_angle);
    target_yaw = now_yaw + delta_angle;

    float tmp_speed = 0, tmp_omega = 0;
    if (distance >= LOCK_CIRCLE_TH)
    {

        BaseChassis.Go2PointStatus.arrived = 0;
        if (BaseChassis.Go2PointStatus.enable_always_yaw_ctrl)
        {
            tmp_omega = PID_GetOutput(&YawPID, target_yaw, BaseChassis.PostureStatus.yaw);
        }
        else
        {
            tmp_omega = 0;
        }
        BaseChassis.target_dir = AngleBetweenPoints(BaseChassis.PostureStatus.x,
                                                    BaseChassis.PostureStatus.y, target.x, target.y);
        tmp_speed = Chassis_Plan2PointSpeed(BaseChassis.Go2PointStatus.start_point, target, start_spd, end_spd, 0.4, 0.4);
        __LIMIT_FROM_TO(tmp_speed, BaseChassis.Go2PointStatus.min_speed, BaseChassis.Go2PointStatus.max_speed);
        // BaseChassis.handbrake_flag = 0;
        // if (TimeFlag_50ms)
        // {
        //     uprintf("distance:%7.3f spd:%6.3f\r\n", distance, tmp_speed);
        // }
    }
    else
    {
        if (distance >= ARRIVED_CIRCIE_TH)
        {
            BaseChassis.Go2PointStatus.arrived = 0;
            tmp_speed = fabs(PID_GetOutput(&LockPID, 0, distance));
            __LIMIT_FROM_TO(tmp_speed, BaseChassis.Go2PointStatus.min_speed, BaseChassis.Go2PointStatus.max_speed);

            tmp_omega = PID_GetOutput(&YawPID, target_yaw, BaseChassis.PostureStatus.yaw);
            BaseChassis.target_dir = AngleBetweenPoints(BaseChassis.PostureStatus.x,
                                                        BaseChassis.PostureStatus.y, target.x, target.y);
            // uprintf("Go2Point|pid circle, spd:%6.3f\r\n", tmp_speed);
        }
        else // 到达
        {
            BaseChassis.Go2PointStatus.arrived = 1;
            // BaseChassis.Go2PointStatus.enable = 0; 到达后希望一直锁定，则不失能
            // uprintf("arrived at %f,%f,%f\r\n", BaseChassis.PostureStatus.x,
            //         BaseChassis.PostureStatus.y, BaseChassis.PostureStatus.yaw);
            tmp_omega = PID_GetOutput(&YawPID, target_yaw, BaseChassis.PostureStatus.yaw);
            tmp_speed = 0;
            // BaseChassis.handbrake_flag = 1;
        }
    }
    BaseChassis.target_speed = tmp_speed;
    if (fabs(delta_angle) < BaseChassis.yaw_ctrl_th)
    {
        tmp_omega = 0;
    }
    if (BaseChassis.Go2PointStatus.disable_yaw_ctrl)
    {
        tmp_omega = 0;
    }
    BaseChassis.target_omega = tmp_omega;
    return;
}

/**
 * @brief 规划两点之间的速度，梯形速度规划
 * 
 * @param start 起点
 * @param target 终点
 * @param start_speed 起点速度，考虑到可能是行驶中调用，速度不为0
 * @param final_speed 终点速度
 * @param acc_ratio 加速系数∈[0,1]，决定梯形规划加速段占比
 * @param dec_ratio 减速系数∈[0,1]
 * @note 请保证acc_ratio+dec_ratio ≤ 1
 * 
 * @return 当前期望速度
 */
float Chassis_Plan2PointSpeed(Point2D_s start, Point2D_s target,
                              float start_speed, float final_speed,
                              float acc_ratio, float dec_ratio)
{
    if (final_speed > start_speed && acc_ratio < 0.05)
    {
        uprintf("## Error! final_speed>start_speed and acc_ratio is too low! ##\r\n");
        return 0;
    }
    float distance_to_target = sqrtf(pow((target.x - BaseChassis.PostureStatus.x), 2) +
                                     pow((target.y - BaseChassis.PostureStatus.y), 2));
    float total_distance = sqrtf(pow((target.x - start.x), 2) + pow((target.y - start.y), 2));
    float distance_offset = fabs(total_distance - distance_to_target); //与起点的距离
    float speed = 0;
    float max_speed = 0.45 * total_distance; // 根据距离缩放速度 0.6 太快
    if (acc_ratio < 1e-4)                    // 纯减速过程
    {
        max_speed = start_speed;
    }
    else if (dec_ratio < 1e-4) // 纯加速过程
    {
        max_speed = final_speed;
    }

    if (distance_offset <= total_distance * acc_ratio && acc_ratio > 1e-4) // 加速过程
    {
        float k = (max_speed - start_speed) / (acc_ratio * total_distance);
        speed = k * distance_offset + start_speed;
    }
    else if (distance_offset <= total_distance * (1 - dec_ratio)) // 匀速过程
    {
        speed = max_speed;
    }
    else if (distance_offset <= total_distance) //减速过程
    {
        float k = -((max_speed - final_speed) / (dec_ratio * total_distance));
        float x = dec_ratio * total_distance - (total_distance - distance_offset);
        float b = final_speed - k * dec_ratio * total_distance;
        speed = k * x + b;
    }
    // uprintf("speed:%.2f distance_to_target:%.2f distance_offset:%.2f\r\n", speed, distance_to_target, distance_offset);
    return speed;
}

void Chassis_Go2Point_Reset()
{
    BaseChassis.Go2PointStatus.start = 0;
    BaseChassis.Go2PointStatus.enable = 0;
    BaseChassis.Go2PointStatus.arrived = 0;
    BaseChassis.Go2PointStatus.start_speed = 0.01;
    BaseChassis.Go2PointStatus.final_speed = 0;
    BaseChassis.Go2PointStatus.disable_yaw_ctrl = 0;
    BaseChassis.Go2PointStatus.enable_always_yaw_ctrl = 1;
    BaseChassis.Go2PointStatus.max_speed = DRIVE_WHEEL_MAX_SPEED;
    BaseChassis.Go2PointStatus.min_speed = DRIVE_WHEEL_MIN_SPEED;
    YawPID.ctrl_max = 5.0;
    BaseChassis.ctrl_mode = CTRL_MODE_NONE;
    BaseChassis.target_speed = 0;
    //速度方向不应更改
    BaseChassis.target_omega = 0;
}

/**
 * @brief 底盘轨迹跟踪中层：计算法向修正向量
 * @param now_speed_vec 当前速度
 * @param target_speed_vec 目标速度
 * @param distance_vec 位移向量
 * @param target_angle 偏航角
 **/
void Chassis_TrackVector(vec now_speed_vec, vec target_speed_vec,
                         vec now_pos2next_target, vec distance_vec, float target_yaw)
{
    if (Vec_IsZero(target_speed_vec))
    {
        BaseChassis.target_omega = 0;
        BaseChassis.target_speed = 0;
    }
    else
    {
        double project = fabs(Vec_DotProduct(now_pos2next_target, distance_vec)) / Vec_Model(now_pos2next_target); //投影长度
        vec project_vec = Vec_ScalarMul(now_pos2next_target, project / Vec_Model(now_pos2next_target));            // 投影向量
        vec corr_vec = Vec_Add(distance_vec, project_vec);                                                         // 法向修正向量
        Vec_ScalarMul(corr_vec, 1.0);                                                                              // 使距离修正与速度大小相关联
        // 对法向偏差进行控制
        float corr_vec_ctrl_x = -PID_GetOutput(&NormalCorrPID_x, 0, corr_vec.x); // 需要使用两个PID结构体，否则last变量会被重用
        float corr_vec_ctrl_y = -PID_GetOutput(&NormalCorrPID_y, 0, corr_vec.y);
        vec corr_vec_ctrl = Vec_Create(corr_vec_ctrl_x, corr_vec_ctrl_y);
        vec velocity_out = Vec_Add(corr_vec_ctrl, target_speed_vec); // 法向修正速度和目标速度合成，得到最终输出速度
        // 偏航角控制
        float delta_angle = target_yaw - BaseChassis.PostureStatus.yaw;
        delta_angle = AngleLimitPI(delta_angle);
        target_yaw = BaseChassis.PostureStatus.yaw + delta_angle;
        float omega_ctrl = PID_GetOutput(&YawPID, target_yaw, BaseChassis.PostureStatus.yaw); // 暂时不加入角速度内环

        // 设置全局目标，交给ChassisMove函数执行
        float speed_out = (float)Vec_Model(velocity_out);
        __LIMIT_FROM_TO(speed_out, 0.01, DRIVE_WHEEL_MAX_SPEED);
        BaseChassis.target_speed = speed_out;
        BaseChassis.target_dir = atan2(velocity_out.y, velocity_out.x);
        BaseChassis.target_omega = omega_ctrl;
        if (TimeFlag_20ms)
        {
            // >>>观测速度<<<
            // float now_speed = sqrt(now_speed_vec.x * now_speed_vec.x + now_speed_vec.y * now_speed_vec.y);
            // int now_dir = (int)RAD2ANGLE(atan2f(now_speed_vec.y, now_speed_vec.x));
            // uprintf("now:%4.2f|%4d|%4.2f target:%4.2f|%4d|%4.2f \r\n",
            //         now_speed, now_dir, BaseChassis.posture_status.yaw,
            //         BaseChassis.target_speed, BaseChassis.target_dir, BaseChassis.target_omega);

            //>>>观测偏航角<<<
            // uprintf("tar_yaw:%.2f now_yaw:%.2f tar_omega:%.2f \r\n", BaseChassis.target_yaw,
            //         BaseChassis.posture_status.yaw,
            //         BaseChassis.target_omega);
        }
    }
}

static vec now_speed_vec;          // 底盘当前的速度向量
static vec target_speed_vec;       // 下一个目标点的速度向量（速度在x、y方向的分量）
static vec now_pos2now_target;     // 底盘当前坐标到当前目标点的位移向量
static vec now_pos2next_target;    // 底盘到下一个目标点的位移向量
static vec now_target2last_target; // 当前目标点到上一个目标点的向量
/**
 * @brief 跟踪轨迹点集
 * @param point_sets 点集数组
 * @param point_num 点的个数
 */
int Chassis_TrackPath(const PlanPoint point_sets[], unsigned int point_num)
{
    // 首次进入
    if (BaseChassis.TrackStatus.start == 1)
    {
        BaseChassis.TrackStatus.start = 0;
        BaseChassis.TrackStatus.point_index = 1; // 使now_pos2next_target为非零值
        uprintf_to(&CHASSIS_MONITOR_UART, "START\r\n");
    }

    if (BaseChassis.TrackStatus.finished) // 轨迹跑完了
    {
        BaseChassis.target_speed = 0;
        BaseChassis.target_omega = 0;
        // Rotate(point_sets[point_num - 1].target_angle); // 进行偏航角微调
        return 1;
    }

    now_speed_vec = Vec_Create(BaseChassis.PostureStatus.speed_x, BaseChassis.PostureStatus.speed_y);
    // 包含轨迹点速度在x，y的分量
    target_speed_vec = Vec_Create((float)point_sets[BaseChassis.TrackStatus.point_index].speed *
                                      cos(point_sets[BaseChassis.TrackStatus.point_index].direct),
                                  (float)point_sets[BaseChassis.TrackStatus.point_index].speed *
                                      sin(point_sets[BaseChassis.TrackStatus.point_index].direct));
    now_pos2now_target = Vec_Create(point_sets[BaseChassis.TrackStatus.point_index].x - BaseChassis.PostureStatus.x,
                                    point_sets[BaseChassis.TrackStatus.point_index].y - BaseChassis.PostureStatus.y);
    now_target2last_target = Vec_Create(point_sets[BaseChassis.TrackStatus.point_index + 1].x - BaseChassis.PostureStatus.x,
                                        point_sets[BaseChassis.TrackStatus.point_index + 1].y - BaseChassis.PostureStatus.y);
    // >>> 注意！这里要求point_index > 0 <<<
    now_pos2next_target = Vec_Create(point_sets[BaseChassis.TrackStatus.point_index - 1].x -
                                         point_sets[BaseChassis.TrackStatus.point_index].x,
                                     point_sets[BaseChassis.TrackStatus.point_index - 1].y -
                                         point_sets[BaseChassis.TrackStatus.point_index].y);
    double distance_to_next = Vec_Model(now_pos2now_target);

    if (TimeFlag_50ms)
    {
        // 通过串口向PC发送轨迹点，用python读取串口信息并绘图
        uprintf_to(&CHASSIS_MONITOR_UART, "track: %.2f %.2f %.2f %.2f\r\n",
                   BaseChassis.PostureStatus.x, BaseChassis.PostureStatus.y,
                   point_sets[BaseChassis.TrackStatus.point_index].x,
                   point_sets[BaseChassis.TrackStatus.point_index].y);

        uprintf("[%d] (%.3f,%.3f)->(%.3f,%.3f) tar_spe:%.2f tar_dir:%.2f\r\n",
                BaseChassis.TrackStatus.point_index,
                BaseChassis.PostureStatus.x, BaseChassis.PostureStatus.y,
                point_sets[BaseChassis.TrackStatus.point_index].x,
                point_sets[BaseChassis.TrackStatus.point_index].y,
                BaseChassis.target_speed, __RAD2ANGLE(BaseChassis.target_dir));
    }
    // 判断是否经过当前目标点
    if (BaseChassis.TrackStatus.point_index < point_num - 1)
    {
        vec forehead = Vec_Create(cos(BaseChassis.target_dir), sin(BaseChassis.target_dir)); // 当前期望速度方向向量
        if (Vec_DotProduct(now_pos2now_target, now_target2last_target) < 0 ||
            Vec_DotProduct(forehead, now_pos2now_target) < 0 || // 经测试，不用距离阈值判断也可以
            distance_to_next <= 0.05)
        {
            BaseChassis.TrackStatus.point_index++;                                  // 指向下一个点
            BaseChassis.Go2PointStatus.start_point.x = BaseChassis.PostureStatus.x; // 除了倒数3个点，这两行赋值应是无效的
            BaseChassis.Go2PointStatus.start_point.y = BaseChassis.PostureStatus.y;
            BaseChassis.Go2PointStatus.enable = 0;
        }
    }
    else // 最后一个点的经过逻辑不同
    {
        if (BaseChassis.Go2PointStatus.arrived)
        {
            BaseChassis.TrackStatus.point_index++;
        }
    }

    if (BaseChassis.TrackStatus.point_index < point_num - 3) // 正常跑点逻辑
    {
        Chassis_TrackVector(now_speed_vec, target_speed_vec, now_pos2next_target, now_pos2now_target,
                            point_sets[BaseChassis.TrackStatus.point_index].target_angle);
    }
    else if (BaseChassis.TrackStatus.point_index == point_num - 3) // 倒数第三个点
    {
        Chassis_TrackVector(now_speed_vec, target_speed_vec, now_pos2next_target, now_pos2now_target,
                            point_sets[BaseChassis.TrackStatus.point_index].target_angle);

        // BaseChassis.Go2PointStatus.start = 0; // 避免一直更新起点
        // BaseChassis.Go2PointStatus.enable = 1;
        // Point2D_s tar;
        // tar.x = point_sets[BaseChassis.TrackStatus.point_index].x;
        // tar.y = point_sets[BaseChassis.TrackStatus.point_index].y;
        // Chassis_Go2Point(tar, point_sets[BaseChassis.TrackStatus.point_index].target_angle,
        //                  0.5, 0.4);
    }
    else if (BaseChassis.TrackStatus.point_index == point_num - 2) // 倒数第二个点
    {
        // Chassis_TrackVector(now_speed_vec, target_speed_vec, now_pos2next_target, now_pos2now_target,
        //                     point_sets[BaseChassis.TrackStatus.point_index].target_angle);
        BaseChassis.Go2PointStatus.start = 0;
        BaseChassis.Go2PointStatus.enable = 1;
        Point2D_s tar;
        tar.x = point_sets[BaseChassis.TrackStatus.point_index].x;
        tar.y = point_sets[BaseChassis.TrackStatus.point_index].y;
        Chassis_Go2Point(tar, point_sets[BaseChassis.TrackStatus.point_index].target_angle,
                         0.5, 0.3);
    }
    else if (BaseChassis.TrackStatus.point_index == point_num - 1) // 最后一个点
    {
        // Chassis_TrackVector(now_speed_vec, target_speed_vec, now_pos2next_target, now_pos2now_target,
        //                     point_sets[BaseChassis.TrackStatus.point_index].target_angle);
        BaseChassis.Go2PointStatus.start = 0;
        BaseChassis.Go2PointStatus.enable = 1;
        Point2D_s tar;
        tar.x = point_sets[BaseChassis.TrackStatus.point_index].x;
        tar.y = point_sets[BaseChassis.TrackStatus.point_index].y;
        Chassis_Go2Point(tar, point_sets[BaseChassis.TrackStatus.point_index].target_angle,
                         0.3, 0);
    }
    else if (BaseChassis.TrackStatus.point_index == point_num) // 轨迹跟踪结束
    {
        BaseChassis.TrackStatus.finished = 1;

        uprintf_to(&CHASSIS_MONITOR_UART, "END\r\n");
        uprintf("--Chassis|Finished tracking path!\r\n");
    }

    return 0;
}

/**
 * @brief 跟踪多个轨迹
 * @param index 轨迹序号
 */
void Chassis_TrackPathSets(int index)
{
    switch (index)
    {
    case -1:
        break;
    case 0:
        BaseChassis.target_speed = 0;
        BaseChassis.target_dir = __ANGLE2RAD(90);
        BaseChassis.target_omega = 0;
        break;
    case 1:
        Chassis_TrackPath(points_pos1, points_pos1_num);
        break;
    case 2:
        Chassis_TrackPath(points_pos2, points_pos2_num);
        break;
    case 3:
        // Chassis_TrackPath(points_pos3, points_pos3_num);
        break;

    default:
        uprintf("\r\n## TraceSets Index Error! ##\r\n");
        break;
    }
}

/**
 * @brief 重置轨迹跟踪状态
 */
void Chassis_TrackStatus_Reset()
{
    BaseChassis.TrackStatus.point_index = 0;
    BaseChassis.TrackStatus.start = 0;
    BaseChassis.TrackStatus.finished = 0;
    BaseChassis.ctrl_mode = CTRL_MODE_NONE;
    YawPID.ctrl_max = 5.0;
    // path_index不应在此函数中更改
    BaseChassis.target_speed = 0;
    //速度方向不应更改
    BaseChassis.target_omega = 0;
}

/**
 * @brief 重映射上位机点集速度参数到底盘可用的数值
 * @param c 转换系数
 * @param max_speed 最大速度
 */
void Chassis_RemappingPathSetsSpeed(PlanPoint point_sets[], int point_num, double k, double b,
                                    float min_speed, float max_speed)
{

    for (int i = 0; i < point_num; i++)
    {
        point_sets[i].speed *= k; // 先放大
        point_sets[i].speed += b; // 再偏移
        if (point_sets[i].speed > max_speed)
        {
            point_sets[i].speed = max_speed;
        }
        if (point_sets[i].speed < min_speed)
        {
            point_sets[i].speed = min_speed;
        }
    }
}

uint8_t Chassis_PrintPostureStatus_Flag = 0; // 手柄或CMD控制开启
void Chassis_PrintPostureStatus()
{
    if (!(Chassis_PrintPostureStatus_Flag && TimeFlag_20ms))
        return;
    uprintf("--ChassisStatus|x:%7.3f y:%7.3f yaw:%5.2f vx:%5.2f vy:%5.2f omega:%5.2f\r\n",
            BaseChassis.PostureStatus.x, BaseChassis.PostureStatus.y,
            BaseChassis.PostureStatus.yaw,
            BaseChassis.PostureStatus.speed_x, BaseChassis.PostureStatus.speed_y,
            BaseChassis.PostureStatus.omega);

    // float scope_arr[3] = {BaseChassis.PostureStatus.speed_x, BaseChassis.PostureStatus.speed_y, BaseChassis.PostureStatus.omega};
    // toolBox_scope(scope_arr, 3);
    // float scope_arr[3] = {BaseChassis.target_omega, BaseChassis.PostureStatus.omega, BaseChassis.omega_ctrl};
    // toolBox_scope(scope_arr, 3);
}

int PrintTargetStatus_Flag = 0;
void Chassis_PrintTargetStatus()
{
    if (!(PrintTargetStatus_Flag && TimeFlag_20ms))
        return;
    uprintf("--target|speed:%.3f dir:%.3f omega:%.3f\r\n",
            BaseChassis.target_speed, BaseChassis.target_dir, BaseChassis.target_omega);
}

char YawTuning_Start = 0;
/**
 * @brief 调试偏航角PID
 * 
 * @param target_yaw 
 */
void Chassis_YawTuning(float target_yaw)
{
    if (!YawTuning_Start)
    {
        BaseChassis.target_speed = 0;
        BaseChassis.target_omega = 0;
        return;
    }
    float now_yaw = BaseChassis.PostureStatus.yaw;
    float delta_angle = target_yaw - now_yaw;
    delta_angle = AngleLimitPI(delta_angle);

    target_yaw = now_yaw + delta_angle;
    float omega = PID_GetOutput(&YawPID, target_yaw, now_yaw);
    if (fabs(delta_angle) < BaseChassis.yaw_ctrl_th)
    {
        omega = 0;
    }

    BaseChassis.target_omega = omega;
    BaseChassis.target_speed = 0;
    if (TimeFlag_20ms)
    {
        uprintf("--Tuing|target_yaw:%.2f now_yaw:%.2f omega:%.2f\r\n", target_yaw, now_yaw, omega);
    }
}

void CAN_Callback_Location_ReadPos_X(CAN_ConnMessage_s *data)
{
    location_raw_x = data->payload.fl[1];
    location_raw_speed_x = data->payload.fl[0];
}

void CAN_Callback_Location_ReadPos_Y(CAN_ConnMessage_s *data)
{
    location_raw_y = data->payload.fl[1];
    location_raw_speed_y = data->payload.fl[0];
}

void CAN_Callback_Location_ReadPos_Yaw(CAN_ConnMessage_s *data)
{
    float yaw = __ANGLE2RAD(data->payload.fl[1]);
    float temp_yaw = yaw + __ANGLE2RAD(90);
    if (temp_yaw < 0)
    {
        temp_yaw += 2 * PI;
    }
    BaseChassis.PostureStatus.yaw = temp_yaw;
    BaseChassis.PostureStatus.omega = __ANGLE2RAD(data->payload.fl[0]);
}