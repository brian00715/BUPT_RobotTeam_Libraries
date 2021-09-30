#ifndef __SW_HANDLE_H_
#define __SW_HANDLE_H_

#include "main.h"
#include "can_utils.h"

#ifdef __cplusplus
extern "C"
{
#endif
//=================================宏和枚举类=================================
#define HANDLE_MAX_LENGTH (130)
#define HANDLE_MIN_LENGTH (20)
#define LEFT_ROCKER_LENGTH_MIN_THRES (20) // 左摇杆幅值低阈值
#define HANDLE_DRIVE_MOTOR_MAX_SPEED (5.0)
#define HANDLE_DRIVE_MOTOR_MIN_SPEED (0)
#define LeftRocker_SpeedTransRatio (144.0f) // 左摇杆矢量长度与底盘速度大小换算比例
#define RightRocker_SpeedTransRatio (2.0f)  // 右摇杆矢量长度与角速度换算比例
#define RELAY_ACCESS GPIO_PIN_SET           // 继电器通
#define RELAY_BROKEN GPIO_PIN_RESET         // 继电器断
    //=================================结构体=================================
    typedef struct HandleRocker_t // 摇杆结构体
    {
        int x;           // 摇杆x坐标
        int y;           // 摇杆y坐标
        uint16_t length; // 油门深度
        float dir;       // 摇杆方向/dir
        int last_length;
        int len_err1; // 用以计算摇杆加速度
        int len_err2;
    } HandleRocker_t;

    struct Handle_t
    {
        HandleRocker_t left_rocker;
        HandleRocker_t right_rocker;
    };

    //=================================全局变量=================================

    extern int SW_Handle_PrintRockerStatus_Flag;

    //=================================函数声明=================================

    void Handle_Init();
    void CAN_Callback_Handle_Button(CAN_ConnMessage_s *data);
    void CAN_Callback_Handle_Rocker(CAN_ConnMessage_s *data);
    void Handle_Exe();
    void Handle_PrintRockerStatus();
    float Handle_RockerMapping(uint16_t amplitude, uint8_t max_amplitude, float max_val, float y0);
#ifdef __cplusplus
}
#endif

#endif