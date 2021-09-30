/*******************************************************************************
 * @brief 手柄控制文件
 * @author Simon
 * @date 2020/11/9
*******************************************************************************/
#include "handle.h"
#include "base_chassis.h"
#include "omni_chassis.h"
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
void CAN_Callback_Handle_Button(CAN_ConnMessage_s *data)
{
    uint8_t id;
    if ((uint8_t)data->payload.ui8[2] == 1)                                   // D2为弹起标志位，按下为1|按键按下时才解析指令
                                                                              //   return;
        id = (uint8_t)((data->payload.ui8[1]) * 10 + (data->payload.ui8[0])); // data的前两位储存id
    else
        return;
    switch (id) // 手柄按钮id
    {
    /*个位系指令用于基础功能*/
    case 0:
        uprintf("--Handle|Switch print handle status!\r\n");
        SW_Handle_PrintRockerStatus_Flag = (SW_Handle_PrintRockerStatus_Flag + 1) % 2;
        break;
    case 1:
        break;
    case 2: // 串口显示定位信息
        PrintChassisStatus_Flag = (PrintChassisStatus_Flag + 1) % 2;
        uprintf("--Print chassis status change to %d\r\n", PrintChassisStatus_Flag);
        break;
    case 3: // 切换坐标模式
        uprintf("--Chassis pos mode changed\r\n");
        BaseChassis.pos_mode = (BaseChassis.pos_mode + 1) % 2;
        break;
    case 4: // 继电器通电
        uprintf("\r\n## Realay has been ACCESSED! Please be careful! ##\r\n");
        HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, RELAY_ACCESS);
        break;
    case 5: // 紧急停止，继电器断电
        uprintf("\r\n## Emergency STOP! Realay has been BROKEN! ##\r\n");
        HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, RELAY_BROKEN);
        break;
    case 6: // 切换控制模式
        break;
    case 7:
        break;
    case 8:
        break;

    case 11:
        break;
    case 12:
        break;
    case 13:
        break;
    case 14:
        break;
    case 15:
        break;

    default:
        break;
    }
}

uint8_t Handle_CANRxOK = 0;
/**
 * @brief 处理手柄摇杆的数据,作为CAN消息中断回调函数
 *        左摇杆的深浅控制速度，方向控制偏航角
 * @param data 手柄摇杆的数据使用int16,can可以一次发送4个
 **/
void CAN_Callback_Handle_Rocker(CAN_ConnMessage_s *data)
{
    Handle.left_rocker.x = (int)data->payload.i16[3];
    Handle.left_rocker.y = (int)data->payload.i16[2];
    Handle.right_rocker.x = (int)data->payload.i16[1];
    Handle.right_rocker.y = (int)data->payload.i16[0];
    Handle_CANRxOK = 1;
}

/**
 * @brief 手柄执行函数,左摇杆控制油门和偏航角；右摇杆控制全场速度
 **/
void Handle_Exe()
{
    if (!Handle_CANRxOK)
    {
        return;
    }
    Handle.left_rocker.length = (uint16_t)sqrt(pow(Handle.left_rocker.x, 2) +
                                               pow(Handle.left_rocker.y, 2));
    Handle.right_rocker.length = (uint16_t)sqrt(pow(Handle.right_rocker.x, 2) +
                                                pow(Handle.right_rocker.y, 2));
    // >>>>>>>>>>>>>>>>>>>>>>>>>>速度矢量控制<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    if (Handle.left_rocker.y > 10)
    {
        BaseChassis.target_speed = (Handle.left_rocker.y - 10) * 1.0 / 85;
    }
    else
    {
        BaseChassis.target_speed = 0;
    }

    // 速度方向
    float temp_frad = atan2(Handle.right_rocker.y, Handle.right_rocker.x);
    Handle.left_rocker.dir = temp_frad; //dir
    // 幅值给定一个阈值，避免摇杆微调时幅角出现剧烈变化
    if (Handle.right_rocker.length > 108)
    {
        BaseChassis.target_dir = temp_frad;
    }

    // >>>>>>>>>>>>>>>>>>>>>>>>偏航角控制<<<<<<<<<<<<<<<<<<<<<<<<<<
    if (fabs(Handle.left_rocker.x) < 20) // 避免油门摇杆归零时车轮呈现自转状态
    {
        BaseChassis.target_omega = 0;
    }
    else
    {
        if (Handle.left_rocker.x > 0)
        {
            BaseChassis.target_omega = -(Handle.left_rocker.x - 20) * 1.0 / 80;
        }
        else
        {
            BaseChassis.target_omega = -(Handle.left_rocker.x + 20) * 1.0 / 80;
        }
    }
    Handle_CANRxOK = 0;
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