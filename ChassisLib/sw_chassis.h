#ifndef __STEER_WHEEL_CHASSIS_
#define __STEER_WHEEL_CHASSIS_
#ifdef USE_RUDDER_CHASSIS
#include "sw_wheel.h"
#include "sw_config.h"
#include "handle.h"
#include "chassis_utils.h"
#include "main.h"
#include "vec.h"
#include "point.h"

#ifdef __cplusplus
extern "C"
{
#endif
    //=================================宏和ENUM=================================

    typedef enum SW_CONTROL_MODE
    {
        SW_MODE_NONE = 0,
        SW_HANDLE,
        SW_CMD,
        SW_TRACK,
        SW_TUNING
    } SW_CONTROL_MODE;

    typedef enum SW_POS_MODE
    {
        SW_RELATIVE = 0, // 机器人坐标系下的相对位姿
        SW_ABSOLUTE      // 世界坐标系下的绝对位姿
    } SW_POS_MODE;

    //=================================结构体=================================
    struct SW_PostureStatus_t // 底盘位姿状态结构体，由全场定位模块更新
    {
        float x; // 单位m
        float y; // 单位m
        float last_x;
        float last_y;
        float yaw; // 偏航角/rad
        float last_yaw;
        float speed;      // 线速度m/s
        float speed_x;    // x方向分速度
        float speed_y;    // y方向分速度
        float omega;      // 角速度rad/s
        float acc;        // 加速度 = Δv2-Δv1 = v2-2v1+v0
        float speed_err2; // Δv2
        float speed_err1; // Δv1
    };

    struct SW_TrackStatus_t // 轨迹跟踪状态结构体
    {
        int begin_track;      // 开始跟踪某一轨迹->状态机置1，标志首次进入pathTrack函数
        int finish_track;     // 完成轨迹标志
        int point_index;      // 轨迹点序号->当前跑轨迹第几个点
        int path_index;       // 轨迹序号->当前跑第几个轨迹
        int points_set_index; // 点集序号->当前跑第几个点
    };

    struct SW_Chassis_t // 底盘抽象结构体
    {
        float target_speed;        // 目标速度大小
        float target_speed_err;    // 目标速度差值
        float target_dir;          // 目标速度方向
        float target_omega;        // 目标角速度
        float target_yaw;          // 目标偏航角
        SW_CONTROL_MODE ctrl_mode; // 控制模式：无/手柄/CMD/自动
        SW_POS_MODE pos_mode;      // 坐标模式：相对/绝对
        struct SW_DriveMotor_t *DriveMotors;
        struct SW_SteerMotor_t *SteerMotors;
        struct SW_PostureStatus_t *PostureStatus;
        struct SW_TrackStatus_t *TrackStatus;
        void (*fChassisMove)(float vel, float dir, float omega); // 底盘控制函数的函数指针
        void (*fWheelsControl)(int index, float vel, float dir); // 单轮控制
    };

    //=================================全局变量=================================

    extern struct SW_Chassis_t SW_Chassis; // 底盘全局结构体

    //=================================函数声明=================================

    void SW_ChassisMove(float vel, float dir, float omega);
    void SW_Init();
    void SW_PostureStatusInit();
    void SW_UpdatePostureStatus();
    void SW_SingleCtrl(int index, float speed, float dir);
    void SW_ChassisCtrlYaw(float yaw, uint32_t max_omega);
    void SW_EXE();
    void SW_TurnMinorArc(float now_dir, float *target_dir, float *target_speed);
    void SW_SuddenStop(float acc);
    void SW_PrintChassisStatus();
    void SW_PrintTargetStatus();
    /**
     * @brief 规划两点间速度，使用梯形速度规划
     * 
     * @param start 起点
     * @param target 终点
     * @param start_speed 起点速度 
     * @param final_speed 终点速度
     * @param max_speed 最大速度
     * @return float 
     */
    float SW_Plan2PointSpeed(Point2D start, Point2D target,
                             float start_speed, float final_speed,
                             float max_speed);

    void SW_TrackVector(vec now_speed_vec, vec target_speed_vec, vec now_pos2next_target, vec distance_vec, float target_angle);
    void SW_TrackPoints(int index);
    /** @brief 路径跟踪状态初始化*/
    void SW_TrackStatusInit();
    int SW_TrackPath(const PlanPoint points_pos[], int point_num);
    void SW_TrackPathSets(int index);
    void SW_ResetTrackStatus();
    void SW_RemappingPathSetsSpeed(PlanPoint point_sets[], int point_num, double k, double b,
                                   float min_speed, float max_speed);
    void SW_YawTuning(float target_yaw);
    void SW_Rotate(float target_yaw);

#ifdef __cplusplus
}
#endif
#endif

#endif