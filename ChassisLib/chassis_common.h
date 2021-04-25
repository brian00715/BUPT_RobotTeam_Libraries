#ifndef CHASSISLIB_H_
#define CHASSISLIB_H_

#include "vec.h"
#include "utils.h"
#include "point.h"

// ===========================选择底盘类型===============================
#define USE_OMNI_CHASSIS
// #define USE_RUDDER_CHASSIS
// #define USE_MECANUM_CHASSIS

typedef enum CHASSIS_POS_MODE
{
    POS_MODE_RELATIVE = 0,
    POS_MODE_ABSOLUTE
} CHASSIS_POS_MODE;

typedef enum CHASSIS_CTRL_MODE
{
    CTRL_MODE_NONE = 0,
    CTRL_MODE_HANDLE = 1,
    CTRL_MODE_CMD,
    CTRL_MODE_TRACK,
    CTRL_MODE_TUNING
} CHASSIS_CTRL_MODE;

typedef struct PostureStatus_t
{
    float x; // 单位m
    float y; // 单位m
    float last_x;
    float last_y;
    float yaw; // 偏航角/rad
    float last_yaw;
    float speed;         // 线速度m/s
    float speed_x;       // x方向分速度
    float speed_y;       // y方向分速度
    float omega;         // 角速度rad/s
    float acc;           // 加速度 = Δv2-Δv1 = v2-2v1+v0
    float speed_err2;    // Δv2
    float speed_err1;    // Δv1
    float laser_pos_x;   //激光获取到的定位
    float laser_pos_y;   //激光获取到的定位
    float laser_pos_yaw; //激光获取到的定位
} PostureStatus_t;

typedef struct TrackStatus_t // 轨迹跟踪状态结构体
{
    int begin_track;      // 开始跟踪某一轨迹->状态机置1，标志首次进入pathTrack函数
    int finish_track;     // 完成轨迹标志
    int point_index;      // 轨迹点序号->当前跑轨迹第几个点
    int path_index;       // 轨迹序号->当前跑第几个轨迹
    int points_set_index; // 点集序号->当前跑第几个点
} TrackStatus_t;

typedef struct BaseChassis_t
{
    float target_speed;
    float target_dir;
    float target_omega;
    PostureStatus_t posture_status;
    TrackStatus_t track_status;
    CHASSIS_POS_MODE pos_mode;
    CHASSIS_CTRL_MODE ctrl_mode;
    void (*fChassisMove)(float target_speed, float target_dir, float target_omega);
} BaseChassis_t;

extern BaseChassis_t BaseChassis;

void Chassis_RemappingPathSetsSpeed(PlanPoint point_sets[], int point_num, double k, double b,
                                    float min_speed, float max_speed);
void Chassis_GoToPoint(Point2D target);
void Chassis_TrackPoints(int index);
float Chassis_Plan2PointSpeed(Point2D start, Point2D target,
                              float start_speed, float final_speed,
                              float max_speed);
void Chassis_TrackVector(vec now_speed_vec, vec target_speed_vec, vec now_pos2next_target, vec distance_vec, float target_yaw);
int Chassis_TrackPath(const PlanPoint point_sets[], int point_num);
void Chassis_TrackPathSets(int index);
void Chassis_ResetTrackStatus();
void PrintChassisStatus();
void PrintTargetStatus();
void Chassis_Init();
void Chassis_PostureStatusInit();
void Chassis_TrackStatusInit();
void SW_UpdatePostureStatus();

#endif