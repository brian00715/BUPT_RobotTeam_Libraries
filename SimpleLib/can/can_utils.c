/*******************************************************************************
 * Copyright:		BUPT
 * File Name:		can_utils.c
 * Description:		CAN 工具函数
 * Author:			ZeroVoid & simon
 * Version:			0.3.0
 * Data:			2021-07-05 15:11:34
 *******************************************************************************/

// TODO: ZeroVoid	due:9/26	错误处理
// TODO: ZeroVoid	due:10/2	动态配置
// TODO: ZeroVoid	due:10/7	优化多中断管理

/**
 * @v0.3.0 将CAN回调函数的通信数据结构改为CAN_ConnMessage_s，使回调函数能够知道DLC RTR等字段
 */

#include "can_utils.h"
#ifdef SLIB_USE_CAN

#include "hash.h"
#include "flags.h"
#include "cmd.h"
#include "can_func.h"
#include <stdlib.h>
// TODO 分离ChassisLib和SimpleLib // #include "motor_driver.h"

/* 全局变量----------------------------------------------------*/
CAN_HandleTypeDef *slib_hcan;
uint8_t CAN_ExeCallback_Flag = 0;
uint8_t CAN_RxCallback_Flag = 0;
static CAN_ConnMessage_s CAN_RxBuffer = {0}; // CAN接收缓冲
CAN_Message_u CAN_TxData;                    // CAN要发送的数据
uint32_t TxMailbox;
CAN_TxHeaderTypeDef CAN_TxHeader;
CAN_TxHeaderTypeDef CAN_ExtTxHeader;
CAN_RxHeaderTypeDef CAN_RxHeader;

/* 局部函数----------------------------------------------------*/
static void CAN_Config(CAN_HandleTypeDef *hcan);
static HashTable CAN_CallbackTable = NULL; // CAN回调函数哈希表
static unsigned int hash_id(const void *id);
static int CAN_IDCmp(const void *, const void *);

void CAN_Init(CAN_HandleTypeDef *hcan)
{
    slib_hcan = hcan;
    CAN_Config(slib_hcan);
    // HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY); //开启发送邮箱空中断
    if (CAN_CallbackTable == NULL)
    {
        CAN_CallbackTable = HashTable_Create(CAN_IDCmp, hash_id, NULL);
    }
    CAN_FuncInit();
}

/**
 * @brief	添加CAN回调函数
 * @param	id          触发回调的can id 
 * @param   callback    回调函数指针 data: can接收到数据联合体
 * @return	None
 */
void CAN_CallbackAdd(const uint32_t id, void (*callback)(CAN_ConnMessage_s *data))
{
    uint32_t *can_id = (uint32_t *)malloc(sizeof(uint32_t));
    *can_id = id;
    HashTable_Insert(CAN_CallbackTable, can_id, (void *)callback);
}

/**
 * @brief 根据CAN ID取回回调函数指针并执行
 */
void CAN_CallbackExe(void)
{
    void (*callback_func)(CAN_ConnMessage_s *) = (void (*)(CAN_ConnMessage_s *))HashTable_GetValue(CAN_CallbackTable, &CAN_RxBuffer.id); // 根据CAN ID取回回调函数
    if (callback_func)
    {
        callback_func(&CAN_RxBuffer); // 执行回调函数
    }
    if (CAN_RxCallback_Flag)
    {
#ifdef SLIB_USE_NRF_COMM
        if (nrf_all_can_send || CAN_RxID == NRF_CAN_SID)
        {
            _can_rx_nrf_callback(&CAN_RxID, &rx_buffer);
        }
#endif // SLIB_USE_NRF_COMM
        CAN_RxCallback(&CAN_RxBuffer);
    }
}

/**
 * @brief can接收中断函数
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_RxHeader, CAN_RxBuffer.payload.ui8);
    CAN_RxBuffer.id = (CAN_RxHeader.IDE == CAN_ID_STD) ? CAN_RxHeader.StdId : CAN_RxHeader.ExtId;
    CAN_RxBuffer.len = CAN_RxHeader.DLC;
    CAN_RxBuffer.rtr = CAN_RxHeader.RTR;
    CAN_ExeCallback_Flag = 1;
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // 再次使能FIFO0接收中断
}

void CAN_SendTest(void)
{
    CAN_TxData.in[0] = 0x1;
    CAN_TxData.in[1] = 0xAD;
    CAN_TxHeader.StdId = 1;
    uprintf("send data\r\n");
    HAL_CAN_AddTxMessage(slib_hcan, &CAN_TxHeader, CAN_TxData.ui8, &TxMailbox);
}

/** 
 * @brief can 发送数据
 * @param id 发送数据id
 * @param msg can数据封装结构体
 * @param len 数据长度
 * @return	0 正常发送
 *          1: 发送失败
 * @todo 无法决定RTR模式,因和其他代码耦合度太高，后续再修改
 **/
int CAN_SendStdMsg(uint16_t std_id, CAN_Message_u *msg)
{
    CAN_TxHeader.StdId = std_id;
    CAN_TxHeader.IDE = CAN_ID_STD;
    CAN_TxHeader.RTR = CAN_RTR_DATA;
    while (HAL_CAN_GetTxMailboxesFreeLevel(slib_hcan) == 0) // 等待发送邮箱空
        HAL_Delay(1);
    if (HAL_CAN_AddTxMessage(slib_hcan, &CAN_TxHeader, msg->ui8, &TxMailbox) != HAL_OK)
    {
        uprintf("Error: CAN can't send msg.\r\n");
        return 1;
    }

    // HAL_CAN_AddTxMessage(&HCAN, &TxHeader, msg->ui8, &TxMailbox); // 发送常规的uint8类型数据
    // uint32_t timecnt = 0;
    // uint32_t can_tsr = 0;
    // for (;;)
    // {
    //     timecnt++;
    //     if (timecnt > 60000)
    //     {
    //         can_tsr = READ_REG(HCAN.Instance->TSR);
    //         if (can_tsr & 0x00020202)
    //         {
    //             return 0;
    //         }
    //         else
    //         {
    //             uprintf("Error: CAN can't send msg.\r\n");
    //             return 1;
    //         }
    //     }
    //  };
    return 0;
}

/**
 * @brief	can ext id send
 * @return	0: send ok; 1: send error;
 */
int CAN_SendExtMsg(uint32_t ext_id, CAN_Message_u *msg)
{
    CAN_TxHeader.ExtId = ext_id;
    CAN_TxHeader.IDE = CAN_ID_EXT;
    CAN_TxHeader.RTR = CAN_RTR_DATA;
    while (HAL_CAN_GetTxMailboxesFreeLevel(slib_hcan) == 0) // 等待发送邮箱空
        HAL_Delay(1);
    if (HAL_CAN_AddTxMessage(slib_hcan, &CAN_TxHeader, msg->ui8, &TxMailbox) != HAL_OK)
    {
        return 1;
    }
    return 0;
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
    return;
}

/**
 * @brief can接收中断回调函数
 * 
 * @param data 
 * @return __weak 
 */
__weak void CAN_RxCallback(CAN_ConnMessage_s *data) {}

void CAN_Config(CAN_HandleTypeDef *hcan)
{

    // FIXME: ZeroVoid	2019/11/13	 len 无法为零, 从而不过滤CAN
    CAN_StdMaskFilterConf(hcan, CAN_AcceptID_Std, sizeof(CAN_AcceptID_Std) / sizeof(CAN_AcceptID_Std[0]), 0);

    //can_std_list_filter_conf(hcan, 325, 0);

    /* Start the CAN peripheral */
    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        /* Start Error */
        Error_Handler();
    }

    /* Activate CAN RX notification */
    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        /* Notification Error */
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_FULL) != HAL_OK)
    {
        /* Notification Error */
        Error_Handler();
    }

    /* Configure Transmission provess */
    CAN_TxHeader.RTR = CAN_RTR_DATA;
    CAN_TxHeader.IDE = CAN_ID_STD;
    CAN_TxHeader.DLC = 8;
    CAN_TxHeader.TransmitGlobalTime = DISABLE;
}

void CAN_StdMaskFilterConf(CAN_HandleTypeDef *hcan, uint32_t *std_id, uint32_t len, uint32_t bank_num)
{
    CAN_FilterTypeDef sFilterConfig;
    uint16_t mask, tmp, i;

    /* Configure the CAN Filter 
     bxCAN提供28个位宽可变/可配置的标识符过滤器组
     通过设置CAN_FMR的FBMx位 设置过滤器类型 0: mask mode; 1: list mode
  */
    sFilterConfig.FilterBank = bank_num;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    /*
    sFilterConfig.FilterIdHigh = (std_id[0]<<5);
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = (0x7ff<<5);
    sFilterConfig.FilterMaskIdLow = (0x0000|0x02);
    */
    /* CAN_FILTERSCALE_32BIT
     FilterIdHigh = StdId << 5 
     FilterIdHigh = (ExtId << 3)>>16 & 0xFFFF
     FIlterIdLow   = ((uint16_t)(ExtId <<3)) | CAN_ID_EXT;*/
    sFilterConfig.FilterIdHigh = len ? (std_id[0] << 5) : 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    mask = 0x7ff;
    for (i = 0; i < len; i++)
    {
        tmp = std_id[i] ^ (~std_id[0]);
        mask &= tmp;
    }
    sFilterConfig.FilterMaskIdHigh = len ? (mask << 5) : 0x0000;
    sFilterConfig.FilterMaskIdLow = (0x0000 | 0x02); // 0x02标准帧 数据帧
    /*
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    */
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
    {
        /* Filter configuration Error */
        Error_Handler();
    }
}

void CAN_StdListFilterConf(CAN_HandleTypeDef *hcan, uint32_t id, uint32_t bank_num)
{
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = bank_num;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = (id << 5);
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = (id << 5);
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
    {
        /* Filter configuration Error */
        Error_Handler();
    }
}

static unsigned int hash_id(const void *id)
{
    return *((unsigned int *)id);
}

static int CAN_IDCmp(const void *a, const void *b)
{
    return (*((unsigned int *)a) == *((unsigned int *)b)) ? 0 : 1;
}

#endif // SLIB_USE_CAN