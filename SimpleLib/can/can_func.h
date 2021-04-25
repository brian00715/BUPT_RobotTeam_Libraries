#ifndef __can_func_H
#define __can_func_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "can_utils.h"
#include "../ChassisLib/handle.h"
#ifdef SL_CAN

    extern int can_data_show_flag;

    void can_func_init();
    void can_show_button(can_msg *data);
    void can_show_rocker(can_msg *data);
    void can_rx_callback(can_msg *data);
    void CAN_Callback_DJI_ReadInfo(can_msg *data);
    void CAN_Callback_DJI_ReadAllPosInfo(can_msg *data);
    void CAN_Callback_VEGA_ReadPos_X(can_msg *data);
    void CAN_Callback_VEGA_ReadPos_Y(can_msg *data);
    void CAN_Callback_VEGA_ReadPos_Yaw(can_msg *data);

#endif // SL_CAN
#ifdef __cplusplus
}
#endif
#endif /*__ can_func_H */