#ifndef DJI_BOARD_CAN_V2_H_
#define DJI_BOARD_CAN_V2_H_

#include "motor_driver.h"

#ifdef USE_MTR_DRIVER_DJI_BOARD_V2

typedef enum
{
    MOTORON = 1,
    MOTOROFF,
    VELCFG,
    POSCFG,
    CURCFG,
    VELCTRL,
    POSCTRL,
    CURCTRL,
    READINFO,
    VELCTRLALL,
    POSCTRLALL,
    HOMING,
    LIMITVELCFG,
    NOINITPOSCFG,
} CANOPTION;


void DJIBoard_MotorOn(CAN_HandleTypeDef *hcan, uint32_t boradId, uint16_t motorId);

void DJIBoard_MotorOff(CAN_HandleTypeDef *hcan, uint32_t boradId, uint16_t motorId);

void DJIBoard_VelCfg(CAN_HandleTypeDef *hcan, uint32_t boradId, uint16_t motorId);

void DJIBoard_LimitVelCfg(CAN_HandleTypeDef *hcan, uint32_t boradId, uint16_t motorId);

void DJIBoard_PosCfg(CAN_HandleTypeDef *hcan, uint32_t boradId, uint16_t motorId, int maxPosVel);

void DJIBoard_NoInitPosCfg(CAN_HandleTypeDef *hcan, uint32_t boradId, uint16_t motorId, int maxPosVel);

void DJIBoard_CurCfg(CAN_HandleTypeDef *hcan, uint32_t boradId, uint16_t motorId);

void DJIBoard_VelCtrl(CAN_HandleTypeDef *hcan, uint32_t boradId, uint16_t motorId, int32_t vel);

void DJIBoard_PosCtrl(CAN_HandleTypeDef *hcan, uint32_t boradId, uint16_t motorId, int32_t pos);

void dji_CurCtrl(CAN_HandleTypeDef *hcan, uint32_t boradId, uint16_t motorId, int32_t cur);
 
void dji_VelCtrlAll(CAN_HandleTypeDef *hcan, uint32_t boradId, int16_t vel[4]);

void dji_PosCtrlAll(CAN_HandleTypeDef *hcan, uint32_t boradId, int16_t pos[4]);

void dji_ReadInfo(CAN_HandleTypeDef *hcan, uint32_t boardId, uint16_t motorId);

void dji_Homing(CAN_HandleTypeDef *hcan, uint32_t boardId, uint16_t motorId, int16_t vel, int16_t cur);

#endif

#endif