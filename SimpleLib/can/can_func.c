
#include "can_func.h"
#include "cmd.h"
#include "handle.h"
#include "dji_ctr.h"
#include "chassis_common.h"
#include "utils.h"

#ifdef SL_CAN

void can_get_mag_mtr(can_msg *data);

void can_func_init()
{
    can_callback_add(324, Handle_Rocker);
    can_callback_add(325, Handle_Button);
    can_callback_add(0x281, CAN_Callback_DJI_ReadInfo);
    can_callback_add(0x282, CAN_Callback_DJI_ReadAllPosInfo);
    can_callback_add(204, CAN_Callback_VEGA_ReadPos_X);
    can_callback_add(205, CAN_Callback_VEGA_ReadPos_Y);
    can_callback_add(206, CAN_Callback_VEGA_ReadPos_Yaw);
}

#ifdef SL_DEBU
can_callback_add(1, can_suc_rx);
can_callback_add(325, can_show_button);
can_callback_add(324, can_show_rocker);
#endif

__weak void can_rx_callback(can_msg *data) {}

void CAN_Callback_DJI_ReadInfo(can_msg *data)
{
    int id = (data->in[0]) >> 16;
    int index = ((data->in[0]) << 16) >> 16;

    if (index == 0x00005850)
    {
        //位置信息
        int num = data->in[1];
        RM_MotorStatus[id].pos = num;
        uprintf("motor %d's pos =%d\r\n", id, num);
    }
    else if (index == 0x00005856)
    {
        //速度信息
        int num = data->in[1];
        RM_MotorStatus[id].rpm = (float)num;
        uprintf("motor %d's vel =%d\r\n", id, num / 1000);
    }
    else if (index == 0x00005149)
    {
        //电流信息
        float num = data->fl[1];
        RM_MotorStatus[id].current = (float)num;
        uprintf("motor %d's cur =%f\r\n", id, num);
    }
}

void CAN_Callback_DJI_ReadAllPosInfo(can_msg *data)
{
    // uprintf("--Received DJI CAN Msg\r\n");
    // 直接得到的是电机输出轴的位置（已经除过819.2了）
    RM_MotorStatus[0].pos = (float)ANGLE2RAD(data->i16[0]);
    RM_MotorStatus[1].pos = (float)ANGLE2RAD(data->i16[1]);
    RM_MotorStatus[2].pos = (float)ANGLE2RAD(data->i16[2]);
    RM_MotorStatus[3].pos = (float)ANGLE2RAD(data->i16[3]);
}

PostureStatus_t BUPT_Location1;
PostureStatus_t BUPT_Location2;
void CAN_Callback_VEGA_ReadPos_X(can_msg *data)
{
    // SW_Chassis.PostureStatus->x = data->fl[0];
    BUPT_Location1.x = data->fl[0];
    BUPT_Location2.x = data->fl[1];
}

void CAN_Callback_VEGA_ReadPos_Y(can_msg *data)
{
    // SW_Chassis.PostureStatus->y = data->fl[0];
    BUPT_Location1.y = data->fl[0];
    BUPT_Location2.y = data->fl[1];
}

void CAN_Callback_VEGA_ReadPos_Yaw(can_msg *data)
{
    // SW_Chassis.PostureStatus->yaw = data->fl[0];
    BUPT_Location1.yaw = data->fl[0];
    BUPT_Location2.yaw = data->fl[1];
}

#endif // SL_CAN