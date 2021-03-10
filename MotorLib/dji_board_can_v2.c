/**
 * @file DJIBoard_board_can_v2.c
 * @author LJunius
 * @brief 大疆驱动板v2（基于OSLib） CAN通信协议相关
 * @version 0.1
 * @date 2021-10-21
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "dji_board_can_v2.h"

#ifdef USE_MTR_DRIVER_DJI_BOARD_V2
#include "oslib_can.h"

void DJIBoard_MotorOn(CAN_HandleTypeDef *hcan, uint32_t boardId, uint16_t motorId)
{
    uint32_t canId = 0x200 + boardId;
    CAN_Message_u msg;
    msg.ui16[0] = motorId;
    msg.ui16[1] = MOTORON;
    msg.in[1] = 0;
    OSLIB_CAN_SendMessage(hcan, 0x00000000U, canId, &msg);
}

void DJIBoard_MotorOff(CAN_HandleTypeDef *hcan, uint32_t boardId, uint16_t motorId)
{
    uint32_t canId = 0x200 + boardId;
    CAN_Message_u msg;
    msg.ui16[0] = motorId;
    msg.ui16[1] = MOTOROFF;
    msg.in[1] = 0;
    OSLIB_CAN_SendMessage(hcan, 0x00000000U, canId, &msg);
}

void DJIBoard_VelCfg(CAN_HandleTypeDef *hcan, uint32_t boardId, uint16_t motorId)
{
    uint32_t canId = 0x200 + boardId;
    CAN_Message_u msg;
    msg.ui16[0] = motorId;
    msg.ui16[1] = VELCFG;
    msg.in[1] = 0;
    OSLIB_CAN_SendMessage(hcan, 0x00000000U, canId, &msg);
}

void DJIBoard_LimitVelCfg(CAN_HandleTypeDef *hcan, uint32_t boardId, uint16_t motorId)
{
    uint32_t canId = 0x200 + boardId;
    CAN_Message_u msg;
    msg.ui16[0] = motorId;
    msg.ui16[1] = LIMITVELCFG;
    msg.in[1] = 0;
    OSLIB_CAN_SendMessage(hcan, 0x00000000U, canId, &msg);
}

void DJIBoard_PosCfg(CAN_HandleTypeDef *hcan, uint32_t boardId, uint16_t motorId, int maxPosVel)
{
    uint32_t canId = 0x200 + boardId;
    CAN_Message_u msg;
    msg.ui16[0] = motorId;
    msg.ui16[1] = POSCFG;
    msg.in[1] = maxPosVel;
    OSLIB_CAN_SendMessage(hcan, 0x00000000U, canId, &msg);
}

void DJIBoard_NoInitPosCfg(CAN_HandleTypeDef *hcan, uint32_t boardId, uint16_t motorId, int maxPosVel)
{
    uint32_t canId = 0x200 + boardId;
    CAN_Message_u msg;
    msg.ui16[0] = motorId;
    msg.ui16[1] = NOINITPOSCFG;
    msg.in[1] = maxPosVel;
    OSLIB_CAN_SendMessage(hcan, 0x00000000U, canId, &msg);
}

void DJIBoard_CurCfg(CAN_HandleTypeDef *hcan, uint32_t boardId, uint16_t motorId)
{
    uint32_t canId = 0x200 + boardId;
    CAN_Message_u msg;
    msg.ui16[0] = motorId;
    msg.ui16[1] = CURCFG;
    msg.in[1] = 0;
    OSLIB_CAN_SendMessage(hcan, 0x00000000U, canId, &msg);
}

void DJIBoard_VelCtrl(CAN_HandleTypeDef *hcan, uint32_t boardId, uint16_t motorId, int32_t vel)
{
    uint32_t canId = 0x200 + boardId;
    CAN_Message_u msg;
    msg.ui16[0] = motorId;
    msg.ui16[1] = VELCTRL;
    msg.in[1] = vel;
    OSLIB_CAN_SendMessage(hcan, 0x00000000U, canId, &msg);
}

void DJIBoard_PosCtrl(CAN_HandleTypeDef *hcan, uint32_t boardId, uint16_t motorId, int32_t pos)
{
    uint32_t canId = 0x200 + boardId;
    CAN_Message_u msg;
    msg.ui16[0] = motorId;
    msg.ui16[1] = POSCTRL;
    msg.in[1] = pos;
    OSLIB_CAN_SendMessage(hcan, 0x00000000U, canId, &msg);
}

void DJIBoard_CurCtrl(CAN_HandleTypeDef *hcan, uint32_t boardId, uint16_t motorId, int32_t cur)
{
    uint32_t canId = 0x200 + boardId;
    CAN_Message_u msg;
    msg.ui16[0] = motorId;
    msg.ui16[1] = CURCTRL;
    msg.in[1] = cur;
    OSLIB_CAN_SendMessage(hcan, 0x00000000U, canId, &msg);
}

/**
 * @brief 信息返回canId为0x280+BOARDID
 * 
 * @param hcan 
 * @param boardId 
 * @param motorId 
 */
void DJIBoard_ReadInfo(CAN_HandleTypeDef *hcan, uint32_t boardId, uint16_t motorId)
{
    uint32_t canId = 0x200 + boardId;
    CAN_Message_u msg;
    msg.ui16[0] = motorId;
    msg.ui16[1] = READINFO;
    msg.in[1] = 0;
    OSLIB_CAN_SendMessage(hcan, 0x00000000U, canId, &msg);
}

void DJIBoard_VelCtrlAll(CAN_HandleTypeDef *hcan, uint32_t boardId, int16_t vel[4])
{
    uint32_t canId = 0x204 + boardId;
    CAN_Message_u msg;
    for (int i = 0; i < 4; i++)
    {
        msg.i16[i] = vel[i];
    }
    OSLIB_CAN_SendMessage(hcan, 0x00000000U, canId, &msg);
}

void DJIBoard_PosCtrlAll(CAN_HandleTypeDef *hcan, uint32_t boardId, int16_t pos[4])
{
    uint32_t canId = 0x208 + boardId;
    CAN_Message_u msg;
    for (int i = 0; i < 4; i++)
    {
        msg.i16[i] = pos[i];
    }
    OSLIB_CAN_SendMessage(hcan, 0x00000000U, canId, &msg);
}

/**
 * @brief Homing
 * 
 * @param hcan 本机can端口
 * @param boardId 大疆板子编号
 * @param motorId 电机编号
 * @param vel 找零速度(编码器线/ms) 建议在100以下,正负代表方向
 * @param cur 找零电流(mA) 建议在0~3000之间
 */
void DJIBoard_Homing(CAN_HandleTypeDef *hcan, uint32_t boardId, uint16_t motorId, int16_t vel, int16_t cur)
{
    uint32_t canId = 0x200 + boardId;
    CAN_Message_u msg;
    msg.ui16[0] = motorId;
    msg.ui16[1] = HOMING;
    msg.i16[2] = vel;
    msg.i16[3] = cur;
    OSLIB_CAN_SendMessage(hcan, 0x00000000U, canId, &msg);
}

#endif