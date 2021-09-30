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
    CTRL_MODE_TUNING,
    CTRL_MODE_EXTERNAL, // 外界直接置位速度和方向
} CHASSIS_CTRL_MODE;

typedef struct PostureStatus_s
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
    float laser_pos_yaw; // 由激光计算出的偏航角
    float pos_corr_x;    // x方向坐标修正量(激光等设备计算得到)
    float pos_corr_y;
} PostureStatus_s;

typedef struct TrackStatus_s // 轨迹跟踪状态结构体
{
    uint8_t start;            // 开始跟踪某一轨迹->状态机置1，标志首次进入pathTrack函数
    uint8_t finished;         // 完成轨迹标志
    uint8_t point_index;      // 轨迹点序号->当前跑轨迹第几个点
    uint8_t track_path_index; // 轨迹序号->当前跑第几个轨迹

} TrackStatus_s;

typedef struct Go2PointStatus_s
{
    uint8_t points_set_index;           // 点集序号->当前跑第几个点
    uint8_t start : 1;                  // 跑点模式启动标志，只存在一瞬间，用于保存起点
    uint8_t arrived : 1;                // 跑点模式到位
    uint8_t enable : 1;                 // 使能，用于安全保护，通常和start一起置位
    uint8_t disable_yaw_ctrl : 1;       // 偏航角控制开关
    uint8_t enable_always_yaw_ctrl : 1; // 开启全程偏航角控制，关闭则在PID锁定范围内才开始调整偏航角
    Point2D_s start_point;
    Point2D_s target_point;
    float target_yaw;
    float start_speed;
    float final_speed;
    float min_speed; // 最小速度
    float max_speed;
} Go2PointStatus_s;

typedef struct CMD_Vel_t
{
    float speed;
    float dir;
    float omega;
} CMD_Vel_t; // 类似ROS中的/cmd_vel

typedef struct BaseChassis_s
{
    float target_speed; // 速度大小
    float target_dir;   // 速度方向
    float target_omega; // 角速度
    float target_yaw;
    float speed_ctrl;  // 里程计外环速度环控制量
    float omega_ctrl;  // 里程计外环偏航角速度控制量
    float yaw_ctrl_th; // 偏航角调整死区
    uint8_t pos_mode;
    uint8_t ctrl_mode;
    uint8_t handbrake_flag : 1; // 手刹标志位，手刹方法由子类实现
    uint8_t lock_yaw_flag : 1;  // 偏航角锁死标志位
    PostureStatus_s PostureStatus;
    TrackStatus_s TrackStatus;
    Go2PointStatus_s Go2PointStatus;
    void (*fChassisMove)(float target_speed, float target_dir, float target_omega);
} BaseChassis_s;

extern BaseChassis_s BaseChassis;

void Chassis_RemappingPathSetsSpeed(PlanPoint point_sets[], int point_num, double k, double b,
                                    float min_speed, float max_speed);
void Chassis_Go2Point(Point2D_s target, float target_yaw, float start_spd, float end_spd);
void Chassis_Go2Point_Reset();
void Chassis_TrackPoints(int index);
float Chassis_Plan2PointSpeed(Point2D_s start, Point2D_s target,
                              float start_speed, float final_speed,
                              float acc_ratio, float dec_ratio);
void Chassis_TrackVector(vec now_speed_vec, vec target_speed_vec, vec now_pos2next_target, vec distance_vec, float target_yaw);
int Chassis_TrackPath(const PlanPoint point_sets[], unsigned int point_num);
void Chassis_TrackPathSets(int index);
void Chassis_TrackStatus_Reset();
void Chassis_PrintPostureStatus();
void Chassis_PrintTargetStatus();
void BaseChassis_Init();
void Chassis_PostureStatusInit();
void Chassis_UpdatePostureStatus();
void Chassis_YawTuning(float target_yaw);
void Chassis_MotionCtrl(void);
void CAN_Callback_Location_ReadPos_X(CAN_ConnMessage_s *data);
void CAN_Callback_Location_ReadPos_Y(CAN_ConnMessage_s *data);
void CAN_Callback_Location_ReadPos_Yaw(CAN_ConnMessage_s *data);

extern BaseChassis_s BaseChassis;
extern Point2D_s CMD_Chassis_TargetPoint;
extern float CMD_Chassis_TargetYaw;
extern float CMD_TargetSpeed;
extern float CMD_TargetDir;
extern float CMD_TargetOmega;
extern float CMD_TargetYaw;
extern char YawTuning_Start;
extern uint8_t Chassis_PrintPostureStatus_Flag;
extern PID_s YawPID;

#endif