/*******************************************************************************
 * Copyright:		BUPT
 * File Name:		can_utils.c
 * Description:		CAN 工具函数
 * Author:			ZeroVoid
 * Version:			0.2.1
 * Data:			2019/09/23 Mon 13:59
 *******************************************************************************/
// TODO: ZeroVoid	due:9/26	错误处理
// TODO: ZeroVoid	due:10/2	动态配置
// TODO: ZeroVoid	due:10/7	优化多中断管理

#include "can_utils.h"
#ifdef SL_CAN

#include "hash.h"
#include "flags.h"
#include "cmd.h"
#include "can_func.h"
#include <stdlib.h>
#include "motor_driver.h"

CAN_HandleTypeDef HCAN;

int can_rx_callback_flag = 0;
CAN_TxHeaderTypeDef TxHeader;
CAN_TxHeaderTypeDef ExtTxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint32_t TxMailbox;
CANMsg can_rx_data;
CANMsg can_tx_data;
uint32_t std_id[] = {230, 324, 325, 0x201, 89, 1, 0x281, 0x282};
// 0x281返回elmo为1的大疆电机的信息，0x282返回四个电机的位置信息
// uint32_t ext_id[] = {0x963};  // 0x963是本杰明电调的状态包ID，第四位是转速

static CANMsg rx_buffer = {0};
static uint32_t rx_id = 0;

static void CAN_Config(CAN_HandleTypeDef *hcan);
static HashTable can_callback_table = NULL;

static unsigned int hash_id(const void *id);
static int id_cmp(const void *, const void *);

void CAN_Init(CAN_HandleTypeDef *hcan)
{
    HCAN = *hcan;
    CAN_Config(&HCAN);
    if (can_callback_table == NULL)
    {
        can_callback_table = HashTable_create(id_cmp, hash_id, NULL);
    }
    CAN_FuncInit();
}

/**
 * @brief	添加CAN回调函数
 * @param	id          触发回调的can id 
 * @param   callback    回调函数指针 data: can接收到数据联合体
 * @return	None
 */
void CAN_CallbackAdd(const uint32_t id, void (*callback)(CANMsg *data))
{
    uint32_t *can_id = (uint32_t *)malloc(sizeof(uint32_t));
    *can_id = id;
    HashTable_insert(can_callback_table, can_id, (void *)callback);
}

/**
 * @brief 所有can消息解析函数的执行体
 */
void CAN_CallbackExe(void)
{
    void (*callback_func)(CANMsg *) = (void (*)(CANMsg *))HashTable_get(can_callback_table, &rx_id);
    if (callback_func)
    {
        callback_func(&rx_buffer); // 执行CAN消息解析函数
    }
    if (can_rx_callback_flag)
    {
#ifdef SL_NRF_COMM
        if (nrf_all_can_send || rx_id == NRF_CAN_SID)
        {
            _can_rx_nrf_callback(&rx_id, &rx_buffer);
        }
#endif // SL_NRF_COMM
        CAN_RxCallback(&rx_buffer);
    }
}

/**
 * @brief can接收中断函数
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, can_rx_data.ui8); // RxHeader属于临时变量
    rx_id = (RxHeader.IDE == CAN_ID_STD) ? RxHeader.StdId : RxHeader.ExtId;
    rx_buffer.df = can_rx_data.df; // copyt can_rx_data to rx_buffer
    if (rx_id == 89)               // 89是本杰明电调反馈状态包的id
    {
        VESC_RxHandler(&can_rx_data);
    }
    can_exc_callback_flag = 1;
}

void can_send_test(void)
{
    can_tx_data.in[0] = 0x1;
    can_tx_data.in[1] = 0xAD;
    TxHeader.StdId = 1;
    uprintf("send data\r\n");
    HAL_CAN_AddTxMessage(&HCAN, &TxHeader, can_tx_data.ui8, &TxMailbox);
}

/** 
 * @brief can 发送数据
 * @param id 发送数据id
 * @param msg can数据封装结构体
 * @param len 数据长度
 * @return	0 正常发送
 *          1: 发送失败
 **/
int CAN_SendMsg(uint16_t std_id, CANMsg *msg)
{
    TxHeader.StdId = std_id;
    TxHeader.IDE = CAN_ID_STD;
#ifdef DEBUG
    uprintf("%d %d %d\r\n", std_id, msg->in[0], msg->in[1]);
#endif                                                            //DEBUG
    HAL_CAN_AddTxMessage(&HCAN, &TxHeader, msg->ui8, &TxMailbox); // 发送常规的uint8类型数据
    //   if (HAL_CAN_AddTxMessage(&HCAN, &TxHeader, msg->ui8, &TxMailbox) != HAL_OK)
    //   {
    //     uprintf("Error: CAN can't send msg.\r\n");
    //     return 1;
    //   }
    uint32_t timecnt = 0;
    uint32_t can_tsr = 0;
    for (;;)
    {
        timecnt++;
        if (timecnt > 60000)
        {
            can_tsr = READ_REG(HCAN.Instance->TSR);
            if (can_tsr & 0x00020202)
            {
                return 0;
            }
            else
            {
                // uprintf("Error: CAN can't send msg.\r\n");
                return 1;
            }
        }
    };
    // return 0;
}

/**
 * @brief	can ext id send
 * @return	0: send ok; 1: send error;
 */
int CAN_SendExtMsg(uint32_t id, CANMsg *msg)
{
    TxHeader.ExtId = id;
    TxHeader.IDE = CAN_ID_EXT;
    if (HAL_CAN_AddTxMessage(&HCAN, &TxHeader, msg->ui8, &TxMailbox) != HAL_OK)
    {
        return 1;
    }
    return 0;
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
    return;
}

__weak void CAN_RxCallback(CANMsg *data) {}

void CAN_Config(CAN_HandleTypeDef *hcan)
{

    // FIXME: ZeroVoid	2019/11/13	 len 无法为零, 从而不过滤CAN
    can_std_mask_filter_conf(hcan, std_id, sizeof(std_id) / sizeof(std_id[0]), 0);

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
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;
}

void can_std_mask_filter_conf(CAN_HandleTypeDef *hcan, uint32_t *std_id, uint32_t len, uint32_t bank_num)
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

void can_std_list_filter_conf(CAN_HandleTypeDef *hcan, uint32_t id, uint32_t bank_num)
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

static int id_cmp(const void *a, const void *b)
{
    return (*((unsigned int *)a) == *((unsigned int *)b)) ? 0 : 1;
}

#endif // SL_CAN