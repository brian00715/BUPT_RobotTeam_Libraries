#ifndef __CAN_UTILS_H
#define __CAN_UTILS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "can.h"
#include "simplelib_cfg.h"
#ifdef SLIB_USE_CAN

    // TODO: ZeroVoid	due:10/28	优化CAN callback,增加保存can数据参数.

    typedef union CAN_Message_u
    {
        char ch[8];
        uint8_t ui8[8];
        uint16_t ui16[4];
        int16_t i16[4];
        int in[2];
        float fl[2];
        double df;
    } CAN_Message_u;

    typedef struct CAN_ConnMessage_s
    {
        uint32_t id; // 标准帧最大 0x7ff, 拓展帧最大 is 0x1FFFFFFF
        uint8_t rtr; // 是否使用远程发送请求
        uint8_t len;
        CAN_Message_u payload;
    } CAN_ConnMessage_s;

    /* slib_hcan允许simplelib其他组件访问, HCAN宏用于兼容旧版本 */
    extern CAN_HandleTypeDef *slib_hcan;
    #define HCAN    (*slib_hcan);
    extern CAN_TxHeaderTypeDef CAN_TxHeader;
    extern CAN_RxHeaderTypeDef CAN_RxHeader;
    extern CAN_TxHeaderTypeDef CAN_ExtTxHeader;
    extern uint32_t TxMailbox;
    extern CAN_Message_u CAN_RxData;
    extern CAN_Message_u CAN_TxData;
    extern uint8_t CAN_ExeCallback_Flag;
    extern uint8_t CAN_RxCallback_Flag;

    void CAN_Init(CAN_HandleTypeDef *hcan);
    int CAN_SendStdMsg(uint16_t std_id, CAN_Message_u *msg);
    int CAN_SendExtMsg(uint32_t ext_id, CAN_Message_u *msg);
    void CAN_CallbackAdd(const uint32_t id, void (*callback)(CAN_ConnMessage_s *data));
    void CAN_CallbackExe(void);
    void CAN_StdMaskFilterConf(CAN_HandleTypeDef *hcan, uint32_t *std_id, uint32_t len, uint32_t bank_num);
    void CAN_StdListFilterConf(CAN_HandleTypeDef *hcan, uint32_t id, uint32_t bank_num);
    void CAN_SendTest(void);
    void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
    void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan);
    __weak void CAN_RxCallback(CAN_ConnMessage_s *data);

#endif // SLIB_USE_CAN

#ifdef __cplusplus
}
#endif

#endif /* __CAN_UTILS_H */