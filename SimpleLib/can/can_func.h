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

    void CAN_FuncInit();
    void can_show_button(CANMsg *data);
    void can_show_rocker(CANMsg *data);
    void CAN_RxCallback(CANMsg *data);
    void CAN_Callback_DJI_ReadInfo(CANMsg *data);
    void CAN_Callback_DJI_ReadAllPosInfo(CANMsg *data);
    void CAN_Callback_VEGA_ReadPos_X(CANMsg *data);
    void CAN_Callback_VEGA_ReadPos_Y(CANMsg *data);
    void CAN_Callback_VEGA_ReadPos_Yaw(CANMsg *data);

#endif // SL_CAN
#ifdef __cplusplus
}
#endif
#endif /*__ can_func_H */