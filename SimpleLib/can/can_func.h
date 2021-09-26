#ifndef __CAN_FUNC_H
#define __CAN_FUNC_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "can_utils.h"
#include "../ChassisLib/handle.h"
#ifdef SL_CAN

    void CAN_FuncInit();

    extern uint32_t CAN_AcceptID_Std[8];

#endif // SL_CAN
#ifdef __cplusplus
}
#endif
#endif /*__ can_func_H */