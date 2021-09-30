/**
 * @file odrive_can.c
 * @author simon
 * @brief ODrive驱动器CAN通信相关
 * @version 0.1
 * @date 2021-07-04
 * @note ODrive驱动板可以同时驱动两个电机.以驱动板为基本对象，每个驱动板的两个axis为子对象
 * @note 本代码基于OOP思想，考虑到事先不知道有几个驱动板或几个电机，设计了用户友好的接口
 * @warning >>>请务必注意ODrive使用的CAN报文ID是否与总线中其他报文的ID有冲突！<<<
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "odrive_can.h"

#ifdef USE_MTR_DRIVER_ODRIVE

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "can.h"
#include "string.h"
#include "can_utils.h"

ODrive_AxisGetState_t ODrive_CAN_RxBuff; // 用于接收ODrive CAN反馈报文的缓冲结构体,配合RxFlag在外部调用
uint8_t ODrive_CAN_RxFlag = 0;           // 一旦接收到ODrive的反馈报文就置1

/**
 * @brief 初始化一个Drive对象，在此设定各项参数
 * @note  需要根据实际情况实现此函数。也可在其他地方直接调用结构体进行初始化
 * 
 * @param odrive  
 * @return uint8_t 
 */
uint8_t ODrive_Init(ODrive_t *odrive)
{
    // 设定示例
    /*
    odrive->set_state[AXIS_0].axis_node_id = ;
    odrive->set_state[AXIS_0].control_mode = ;
    odrive->set_state[AXIS_0].input_current = ;
    odrive->set_state[AXIS_0].input_vel = ;
    odrive->set_state[AXIS_0].input_pos = ;
    odrive->set_state[AXIS_0].traj_accel_limit = ;
    odrive->set_state[AXIS_0].traj_decel_limit = ;
    odrive->set_state[AXIS_0].traj_vel_limit = ;
    odrive->set_state[AXIS_0].vel_limit = ;
    odrive->set_state[AXIS_0].current_limit = ;

    odrive->set_state[AXIS_1].axis_node_id = ;
    odrive->set_state[AXIS_1].control_mode = ;
    odrive->set_state[AXIS_1].input_current = ;
    odrive->set_state[AXIS_1].input_vel = ;
    odrive->set_state[AXIS_1].input_pos = ;
    odrive->set_state[AXIS_1].traj_accel_limit = ;
    odrive->set_state[AXIS_1].traj_decel_limit = ;
    odrive->set_state[AXIS_1].traj_vel_limit = ;
    odrive->set_state[AXIS_1].vel_limit = ;
    odrive->set_state[AXIS_1].current_limit = ;
    */

    /** @todo 测试不配置这些过滤能否正常使用 */
    // CAN_FilterTypeDef filter;
    // filter.FilterActivation = ENABLE;
    // filter.FilterBank = 0;
    // filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    // filter.FilterIdHigh = 0x0000;
    // filter.FilterIdLow = 0x0000;
    // filter.FilterMaskIdHigh = 0x0000;
    // filter.FilterMaskIdLow = 0x0000;
    // filter.FilterMode = CAN_FILTERMODE_IDMASK;
    // filter.FilterScale = CAN_FILTERSCALE_32BIT;
    // HAL_StatusTypeDef status = HAL_CAN_ConfigFilter(&hcan1, &filter);
    // if (status != HAL_OK)
    // {
    //     /* Start Error */
    //     Error_Handler();
    //     return 1;
    // }
    // status = HAL_CAN_Start(&hcan1);
    // if (status != HAL_OK)
    // {
    //     /* Start Error */
    //     Error_Handler();
    //     return 1;
    // }
    // HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    return 0;
}

/**
 * @brief 解析ODrive发回的报文
 * 
 * @param axis ODrive电机类对象的指针
 * @param msg 从CAN总线中接收到的报文
 * @note 该函数需要直接放在CAN接收中断回调函数中，与simplelib的CAN回调执行不兼容；也可根据电机的数量在外部分别写对应的回调函数，利用全局变量填充数据。 
 * @todo 利用C语言未知参数机制解决和simplelib不兼容的问题
 * @return uint8_t 
 */
uint8_t CAN_Callback_ODriveAxis(ODriveAxis_t *axis, CAN_ConnMessage_s *msg)
{
    uint8_t first_word[4];
    uint8_t second_word[4];
    if (msg->len == 8)
    {
        memcpy(&first_word, &msg->payload.ui8, 4);
        second_word[3] = msg->payload.ui8[7];
        second_word[2] = msg->payload.ui8[6];
        second_word[1] = msg->payload.ui8[5];
        second_word[0] = msg->payload.ui8[4];
    }
    else if (msg->len == 4)
    {
        memcpy(&first_word, &msg->payload.ui8, 4);
    }

    if (msg->rtr == CAN_RTR_DATA) // 是从ODrive发回的报文
    {
        switch (msg->id & 0x11111) // 过滤掉电机ID，保留控制ID
        {
        case (ODRIVE_MSG_ODRIVE_HEARTBEAT):
            memcpy(&axis->get_state.axis_error, &first_word, sizeof(uint32_t));
            memcpy(&axis->get_state.axis_current_state, &second_word, sizeof(uint32_t));
            break;
        case (ODRIVE_MSG_GET_ENCODER_ESTIMATES):
            memcpy(&axis->get_state.encoder_pos_estimate, &first_word, sizeof(float));
            memcpy(&axis->get_state.encoder_vel_estimate, &second_word, sizeof(float));
            break;
        case (ODRIVE_MSG_GET_ENCODER_COUNT):
            memcpy(&axis->get_state.encoder_shadow_count, &first_word, sizeof(uint32_t));
            memcpy(&axis->get_state.encoder_cpr_count, &second_word, sizeof(uint32_t));
            break;
        case (ODRIVE_MSG_GET_MOTOR_ERROR):
            memcpy(&axis->get_state.motor_error, &first_word, sizeof(uint32_t));
            break;
        case (ODRIVE_MSG_GET_ENCODER_ERROR):
            memcpy(&axis->get_state.encoder_error, &first_word, sizeof(uint32_t));
            break;
        case (ODRIVE_MSG_GET_VBUS_VOLTAGE):
            memcpy(&axis->get_state.vbus_voltage, &first_word, sizeof(float));
            break;
        default:
            return 1;
            break;
        }
    }
    return 0;
}

/**
 * @brief 向ODrive发送控制CAN报文
 * 
 * @param odrive 驱动器对象
 * @param axis 电机序号
 * @param msg 控制命令，可选参数见odrive_can.h
 * @return uint8_t 
 */
uint8_t ODrive_CANSendMsg(CAN_HandleTypeDef *hcan, ODrive_t *odrive, uint8_t axis, ODrive_CAN_CmdMsg_e msg)
{
    CAN_TxHeaderTypeDef header;
    header.IDE = CAN_ID_STD;
    uint8_t data[8];
    uint8_t tmp_word[4]; // TODO: Get rid of this with better mem management
    ODrive_AxisSetState_t *odrive_set;
    if (axis == AXIS_0)
    {
        header.StdId = odrive->axis0.mtr_id << 5 | msg;
        odrive_set = &odrive->axis0.set_state;
    }
    else if (axis == AXIS_1)
    {
        header.StdId = odrive->axis1.mtr_id << 5 | msg;
        odrive_set = &odrive->axis1.set_state;
    }
    else
    {
        return -1;
    }

    switch (msg)
    {
    case ODRIVE_MSG_ODRIVE_ESTOP:
        /* TODO: Implement */
        break;
    case ODRIVE_MSG_GET_MOTOR_ERROR:
        header.RTR = CAN_RTR_REMOTE;
        header.DLC = 0;
        break;
    case ODRIVE_MSG_GET_ENCODER_ERROR:
        header.RTR = CAN_RTR_REMOTE;
        header.DLC = 0;
        break;
    case ODRIVE_MSG_GET_SENSORLESS_ERROR:
        /* TODO: Implement */
        break;
    case ODRIVE_MSG_SET_AXIS_NODE_ID:
        /* TODO: Implement */
        break;
    case ODRIVE_MSG_SET_AXIS_REQUESTED_STATE:
        memcpy(&data, &odrive_set->requested_state, 4);
        header.RTR = CAN_RTR_DATA;
        header.DLC = 4;
        break;
    case ODRIVE_MSG_SET_AXIS_STARTUP_CONFIG:
        /* TODO: Implement */
        break;
    case ODRIVE_MSG_GET_ENCODER_ESTIMATES:
        header.RTR = CAN_RTR_REMOTE;
        header.DLC = 0;
        break;
    case ODRIVE_MSG_GET_ENCODER_COUNT:
        header.RTR = CAN_RTR_REMOTE;
        header.DLC = 0;
        break;
    case ODRIVE_MSG_SET_CONTROLLER_MODES:
        data[0] = odrive_set->control_mode;
        data[4] = odrive_set->input_mode;
        header.RTR = CAN_RTR_DATA;
        header.DLC = 8;
        break;
    case ODRIVE_MSG_SET_INPUT_POS:
        memcpy(&data, &odrive_set->input_pos, 4);
        data[4] = odrive_set->vel_ff & 0x00FF;
        data[5] = odrive_set->vel_ff >> 8;
        data[6] = odrive_set->torque_ff & 0x00FF;
        data[7] = odrive_set->torque_ff >> 8;
        header.RTR = CAN_RTR_DATA;
        header.DLC = 8;
        break;
    case ODRIVE_MSG_SET_INPUT_VEL:
        memcpy(&data, &odrive_set->input_vel, 4);
        data[4] = odrive_set->torque_ff & 0x00FF;
        data[5] = odrive_set->torque_ff >> 8;
        header.RTR = CAN_RTR_DATA;
        header.DLC = 6;
        break;
    case ODRIVE_MSG_SET_INPUT_CURRENT:
        memcpy(&data, &odrive_set->input_current, 4);
        header.RTR = CAN_RTR_DATA;
        header.DLC = 4;
        break;
    case ODRIVE_MSG_SET_VEL_LIMIT:
        memcpy(&data, &odrive_set->vel_limit, 4);
        header.RTR = CAN_RTR_DATA;
        header.DLC = 4;
        break;
    case ODRIVE_MSG_START_ANTICOGGING:
        header.RTR = CAN_RTR_REMOTE;
        header.DLC = 0;
        break;
    case ODRIVE_MSG_SET_TRAJ_VEL_LIMIT:
        memcpy(&data, &odrive_set->traj_vel_limit, 4);
        header.RTR = CAN_RTR_DATA;
        header.DLC = 4;
        break;
    case ODRIVE_MSG_SET_TRAJ_ACCEL_LIMITS:
        memcpy(&data, &odrive_set->traj_accel_limit, 4);
        memcpy(&tmp_word, &odrive_set->traj_decel_limit, 4);
        data[4] = tmp_word[0];
        data[5] = tmp_word[1];
        data[6] = tmp_word[2];
        data[7] = tmp_word[3];
        header.RTR = CAN_RTR_DATA;
        header.DLC = 4;
        break;
    case ODRIVE_MSG_SET_TRAJ_A_PER_CSS:
        memcpy(&data, &odrive_set->traj_a_per_css, 4);
        header.RTR = CAN_RTR_DATA;
        header.DLC = 4;
        break;
    case ODRIVE_MSG_GET_IQ:
        /* TODO: Implement */
        break;
    case ODRIVE_MSG_GET_SENSORLESS_ESTIMATES:
        /* TODO: Implement */
        break;
    case ODRIVE_MSG_RESET_ODRIVE:
        header.RTR = CAN_RTR_REMOTE;
        header.DLC = 0;
        break;
    case ODRIVE_MSG_GET_VBUS_VOLTAGE:
        header.RTR = CAN_RTR_REMOTE;
        header.DLC = 0;
        break;
    case ODRIVE_MSG_CLEAR_ERRORS:
        header.RTR = CAN_RTR_REMOTE;
        header.DLC = 0;
        break;
    case ODRIVE_MSG_CO_HEARTBEAT_CMD:
        /* TODO: Implement */
        break;
    default:
        break;
    }

    uint32_t retTxMailbox = 0;
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
    {
        HAL_CAN_AddTxMessage(hcan, &header, data, &retTxMailbox);
    }

    return 1;
}

#endif