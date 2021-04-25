/*******************************************************************************
 * @brief 舵轮手柄控制文件
 * @author Simon
 * @date 2020/11/9
*******************************************************************************/
#include "handle.h"
#include "chassis_common.h"
#include "utils.h"
#include "cmd.h"

struct Handle_t Handle; // 手柄数据结构体，包含摇杆位置数据、模式等

void Handle_Init()
{
    Handle.right_rocker.y = 0;
    Handle.right_rocker.x = 0;
    Handle.left_rocker.y = 0;
    Handle.left_rocker.x = 0;
    Handle.left_rocker.last_length = 0;
    Handle.left_rocker.len_err1 = 0;
    Handle.left_rocker.len_err2 = 0;
}

extern int PrintChassisStatus_Flag;
/**
 * @brief 处理手柄按键数据，作为CAN消息中断回调函数
 * 
 * @param data 
 */
void Handle_Button(can_msg *data)
{
    uint8_t id;
    if ((uint8_t)data->ui8[2] == 1)                           // D2为弹起标志位，按下为1|按键按下时才解析指令
                                                              //   return;
        id = (uint8_t)((data->ui8[1]) * 10 + (data->ui8[0])); // data的前两位储存id
    else
        return;
    switch (id)
    {
    /*个位系指令用于基础功能*/

    /*10系指令用于底盘轨迹跟踪控制*/

    /*20系指令用于底盘参数控制*/
    case 21:
        break;
    case 22:
        break;
    case 23:
        break;

    default:
        break;
    }
}

/**
 * @brief 处理手柄摇杆的数据,作为CAN消息中断回调函数
 *        左摇杆的深浅控制速度，方向控制偏航角
 * @param data 手柄摇杆的数据使用int16,can可以一次发送4个
 **/
void Handle_Rocker(can_msg *data)
{
    Handle.left_rocker.x = (int)data->i16[3];
    Handle.left_rocker.y = (int)data->i16[2];
    Handle.right_rocker.x = (int)data->i16[1];
    Handle.right_rocker.y = (int)data->i16[0];
}

static float RROCKER_ZERO_OFFSET = 0.314; // ±10°内无值，从而让手指可以一直顶着摇杆，避免误操作
static float target_speed_ar[5] = {0};    // 驱动电机历史速度，用以滤波
static int target_speed_ar_index = 0; // 数组的指针
/**
 * @brief 手柄执行函数,左摇杆控制速度矢量，深度控制速度大小；右摇杆控制偏航角，与y轴的偏差角度大小控制角速度
 **/
void SW_Handle_EXE()
{
    Handle.left_rocker.length = (uint16_t)sqrt(pow(Handle.left_rocker.x, 2) +
                                               pow(Handle.left_rocker.y, 2));
    Handle.right_rocker.length = (uint16_t)sqrt(pow(Handle.right_rocker.x, 2) +
                                                pow(Handle.right_rocker.y, 2));
    Handle.left_rocker.len_err1 = Handle.left_rocker.length - Handle.left_rocker.last_length;
    Handle.left_rocker.last_length = Handle.left_rocker.length;

    // >>>>>>>>>>>>>>>>>>>>>>>>>>速度矢量控制<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    // 速度大小
    if (Handle.left_rocker.length < LEFT_ROCKER_LENGTH_MIN_THRES) // 幅值要足够大才能允许有效，因为幅值较小时稍微一动相角就会发生巨大变化
    {
        Handle.left_rocker.length = 0; // 解决零漂导致的静止速度不为0
    }
    else
    {
        Handle.left_rocker.length -= LEFT_ROCKER_LENGTH_MIN_THRES;
    }
    float raw_speed = Handle_RockerMapping(Handle.left_rocker.length,
                                           (uint8_t)128, 5.0f, 0);
    float speed_diff = raw_speed - fabs(BaseChassis.target_speed); // 与上一次目标速度大小的差值
    // 使速度不连续变化，避免电机抽搐;同时使速度变化产生延迟，降低加速度
    // if (speed_diff > 800)                                           // 加速
    // {
    //     SW_Chassis.target_speed += 800;
    // }
    // else if (speed_diff < -800) // 减速
    // {
    //     SW_Chassis.target_speed -= 800;
    // }
    // if (fabs(speed_diff) > 0.2)
    // {
    //     SW_Chassis.target_speed = raw_speed;
    // }
    BaseChassis.target_speed = raw_speed;
    target_speed_ar[(target_speed_ar_index++) % 5] = BaseChassis.target_speed;
    float ave_result;
    AVE_OF_AR(target_speed_ar, 5, ave_result); // 对输出速度进行均值滤波
    BaseChassis.target_speed = ave_result;
    // if (speed_diff <= -1000)                                   // 减速过于剧烈时启动急刹
    // {
    //     SW_SuddenStop((90 / 1000) * fabs(speed_diff));
    //     uprintf("--sudden stop!\r\n");
    // }

    // 速度方向
    float temp_frad = atan2(Handle.left_rocker.y, Handle.left_rocker.x);
    Handle.left_rocker.dir = temp_frad; //dir
    // 幅值给定一个阈值，避免摇杆微调时幅角出现剧烈变化，且收油门的时候停止控制舵向
    if (Handle.left_rocker.length > LEFT_ROCKER_LENGTH_MIN_THRES &&
        !(speed_diff < 0 && Handle.left_rocker.length < LEFT_ROCKER_LENGTH_MIN_THRES))
    {
        //     // float delta_angle = temp_frad - SW_Chassis.target_dir; // 期望角度与当前角度的偏差
        //     // if (abs(delta_angle) > 2)                                  // 每n°改变一次，避免抽抽
        {
            BaseChassis.target_dir = temp_frad;
        }
    }

    // >>>>>>>>>>>>>>>>>>>>>>>>偏航角控制<<<<<<<<<<<<<<<<<<<<<<<<<<
    float temp_fturn;
    float angle_offset;
    if (Handle.right_rocker.length > 108) // 只有摇杆顶到头才开启转动
    {
        // 计算右摇杆相角与与pi/2的偏差值(rad)，大小控制角速度
        angle_offset = AngleLimitDiff(atan2(Handle.right_rocker.y, Handle.right_rocker.x), PI / 2);
        if (fabs(angle_offset) < RROCKER_ZERO_OFFSET)
        {
            angle_offset = 0;
        }
        else if (angle_offset > 0)
        {
            angle_offset -= RROCKER_ZERO_OFFSET;
        }
        else if (angle_offset < 0)
        {
            angle_offset += RROCKER_ZERO_OFFSET;
        }
        temp_fturn = RightRocker_SpeedTransRatio * angle_offset;
    }
    else
    {
        temp_fturn = 0;
    }
    // 控制加速度
    // float fturn_diff = temp_fturn - SW_Chassis.target_omega;
    // if (fturn_diff > 0.2)
    // {
    //     SW_Chassis.target_omega += 0.2;
    // }
    // else if (fturn_diff < 0.2)
    // {
    //     SW_Chassis.target_omega -= 0.2;
    // }
    // else
    // {
    //     SW_Chassis.target_omega = temp_fturn;
    // }
    BaseChassis.target_omega = temp_fturn;

    // uprintf("--target_dir=%d target_omega=%.2f \r\n", SW_Chassis.target_dir, SW_Chassis.target_omega);
    // uprintf("--targetSpeed:%.3f targetDir:%d\r\n", SW_Chassis.target_speed, SW_Chassis.target_dir);
}

int SW_Handle_PrintRockerStatus_Flag = 0;
/**
 * @brief 串口打印摇杆信息
 */
void SW_Handle_PrintRockerStatus()
{
    if (!(SW_Handle_PrintRockerStatus_Flag && TimeFlag_20ms))
        return;
    uprintf("--lx:%4d ly:%4d rx:%4d ry:%4d | ", Handle.left_rocker.x, Handle.left_rocker.y,
            Handle.right_rocker.x, Handle.right_rocker.y);
    uprintf("llen:%3d ldir:%.2f rlen:%3d rdir:%.2f\r\n", Handle.left_rocker.length, Handle.left_rocker.dir,
            Handle.right_rocker.length, Handle.right_rocker.dir);
}

/**
 * @brief 手柄摇杆幅值与输出值（例如速度）的映射关系
 * @param amplitude 需要映射的幅值
 * @param max_amplitude 摇杆最大幅值
 * @param max_val 映射后的最大值
 * @param y0 常数补偿
 * @retval 映射后的数值
 * @note 目前采用二次方映射
 */
float Handle_RockerMapping(uint16_t amplitude,
                           uint8_t max_amplitude,
                           float max_val,
                           float b)
{
    // return (1.125 * amplitude * amplitude - 1012);

    // float a = max_val / (max_amplitude * max_amplitude); // y=ax^2+b
    // return a * (amplitude * amplitude) + b;

    // float a = max_val / max_amplitude; // y = ax+b
    // return (a * amplitude + b);

    float x = (-max_amplitude * 1.0 / 2.0 + amplitude * 1.0) * 8.0 / 65.0; // sigmoid函数
    float a = 1.0 / (1.0 + exp(-x));
    return a * max_val + b;
}