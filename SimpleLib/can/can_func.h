#ifndef __CAN_FUNC_H
#define __CAN_FUNC_H
#ifdef __cplusplus
extern "C"
{
#endif

#ifdef SLIB_USE_CAN

    void CAN_FuncInit();
    extern uint32_t CAN_AcceptID_Std[8];

#endif // SLIB_USE_CAN
#ifdef __cplusplus
}
#endif
#endif /*__ can_func_H */