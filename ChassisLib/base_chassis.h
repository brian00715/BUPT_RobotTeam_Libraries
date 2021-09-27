#ifndef BASE_CHASSIS_H_
#define BASE_CHASSIS_H_

#include "vec.h"
#include "utils.h"
#include "point.h"
#include "stm32f4xx.h"
#include "can_utils.h"

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
    CTRL_MODE_GO_TO_POINT,
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
    uint8_t begin_track;      // 开始跟踪某一轨迹->状态机置1，标志首次进入pathTrack函数
    uint8_t finished_track;   // 完成轨迹标志
    uint8_t point_index;      // 轨迹点序号->当前跑轨迹第几个点
    uint8_t path_index;       // 轨迹序号->当前跑第几个轨迹
    uint8_t points_set_index; // 点集序号->当前跑第几个点
    uint8_t go2point_start;   // 跑点模式启动标志
} TrackStatus_t;

typedef struct CMD_Vel_t
{
    float speed;
    float dir;
    float omega;
} CMD_Vel_t; // 类似ROS中的/cmd_vel

typedef struct BaseChassis_t
{
    float target_speed;
    float target_dir;
    float target_omega;
    float speed_ctrl; // 速度环控制量
    float omega_ctrl;
    uint8_t pos_mode;
    uint8_t ctrl_mode;
    uint8_t handbrake_flag; // 手刹标志位，手刹方法由子类实现
    PostureStatus_t PostureStatus;
    TrackStatus_t TrackStatus;
    void (*fChassisMove)(float target_speed, float target_dir, float target_omega);
} BaseChassis_t;

extern BaseChassis_t BaseChassis;

void Chassis_RemappingPathSetsSpeed(PlanPoint point_sets[], int point_num, double k, double b,
                                    float min_speed, float max_speed);
void Chassis_Go2Point(Point2D_s target, float target_yaw);
void Chassis_TrackPoints(int index);
float Chassis_Plan2PointSpeed(Point2D_s start, Point2D_s target,
                              float start_speed, float final_speed,
                              float acc_ratio, float dec_ratio);
void Chassis_TrackVector(vec now_speed_vec, vec target_speed_vec, vec now_pos2next_target, vec distance_vec, float target_yaw);
int Chassis_TrackPath(const PlanPoint point_sets[], unsigned int point_num);
void Chassis_TrackPathSets(int index);
void Chassis_ResetTrackStatus();
void PrintChassisStatus();
void PrintTargetStatus();
void BaseChassis_Init();
void Chassis_PostureStatusInit();
void Chassis_TrackStatusInit();
void Chassis_UpdatePostureStatus();
void Chassis_YawTuning(float target_yaw);
void Chassis_MotionCtrl(void);
void CAN_Callback_Locator_ReadPos_X(CAN_ConnMessage_s *data);
void CAN_Callback_Locator_ReadPos_Y(CAN_ConnMessage_s *data);
void CAN_Callback_Locator_ReadPos_Yaw(CAN_ConnMessage_s *data);

extern BaseChassis_t BaseChassis;
extern float CMD_TargetSpeed;
extern float CMD_TargetDir;
extern float CMD_TargetOmega;
extern float CMD_TargetYaw;
extern char YawTuning_Start;
extern Point2D_s CMD_Chassis_TargetPoint;
extern float CMD_Chassis_TargetYaw;

#endif