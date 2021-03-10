#ifndef DJI_BOARD_CAN_V1_H_
#define DJI_BOARD_CAN_V1_H_

#include "stm32f4xx.h"
#include "main.h"

//ELMO驱动器CAN广播ID号
#define ELMO_BROADCAST_ID (0x000)
//ELMO驱动器接收ID基址
#define ELMO_DEVICE_BASEID (0x300)
//ELMO驱动器发送ID基址
#define SDO_RESPONSE_COB_ID_BASE 0x280

/******************驱动器工作模式************************/
#define SPEED_CONTROL_MODE (2)
#define POSITION_CONTROL_MODE (5)
#define HOMING_MODE (6)
#define CUR_CONTROL_MODE (7)
/*********************位置环运行模式**********************/
#define ABSOLUTE_MODE (0)

/** 
  * @brief  电机种类  3508 or 2006
  */
typedef enum
{
	RM_3508 = 1,
	M_2006 = 2,
	NONE = 3 //none表示没有接电机
} MotorType_TypeDef;

/** 
  * @brief  Robomaster Motor 3508/2006 type structure definition  
  * @note     
  */
typedef struct
{
	int32_t pos, posLast;
	MotorType_TypeDef type;
	int32_t vel; //电机返回的速度，单位： rpm
	int16_t cur; //电机返回的电流值
	int8_t temp; //电机的温度
} MotorType;

typedef struct RM_MotorStatus_t
{
	float pos; // rad
	float rpm;
	float current;
} RM_MotorStatus_t; // 配合东大驱动板使用的状态结构体

extern RM_MotorStatus_t RM_MotorStatus[4];
extern MotorType Motor[8];

/**
* @brief  电机使能（通电）
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION(migrate by LJ)
* @note ELMO驱动器默认初始状态为电机失能，使用电机时需要对其进行使能
*       部分驱动器参数需要在电机失能状态下才可以配置
*/
void DJIBoard_MotorOn(CAN_TypeDef *CANx, uint8_t ElmoNum);

/**
* @brief  电机失能（断电）
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION(migrate by LJ)
*/
void DJIBoard_MotorOff(CAN_TypeDef *CANx, uint8_t ElmoNum);

/**
* @brief  驱动器速度环初始化
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  acc：加速度，单位：脉冲每二次方秒
* @param  dec：减速度，单位：脉冲每二次方秒
* @author ACTION(migrate by LJ)
* @note 在速度环初始化后才可以使能电机！！
*/
void DJIBoard_VelLoopCfg(CAN_TypeDef *CANx, uint8_t ElmoNum, uint32_t acc, uint32_t dec);

/**
* @brief  驱动器位置环初始化
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  acc：加速度，单位：脉冲每二次方秒
* @param  dec：减速度，单位：脉冲每二次方秒
* @param  vel: 速度，单位：脉冲每秒，范围：最小速度限制到最大速度限制
* @author ACTION(migrate by LJ)
* @note 在位置环初始化后才可以使能电机！！
*/
void DJIBoard_PosLoopCfg(CAN_TypeDef *CANx, uint8_t ElmoNum, uint32_t acc, uint32_t dec, uint32_t vel);

/**
* @brief  驱动器电流环初始化
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author LJ
* @note 在电流环初始化后才可以使能电机！！
*/
void DJIBoard_CurLoopCfg(CAN_TypeDef *CANx, uint8_t ElmoNum);

/**
* @brief  电机速度控制
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  vel: 速度，单位：脉冲每秒，范围：最小速度限制到最大速度限制
* @author ACTION(migrate by LJ)
*/
void DJIBoard_VelCrl(CAN_TypeDef *CANx, uint8_t ElmoNum, int32_t vel);

/**
* @brief  电机位置控制
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  posMode: 位置环运行模式，范围：
				ABSOLUTE_MODE: 绝对位置模式
				RELATIVE_MODE: 相对位置模式
* @param  pos:位置命令，单位：脉冲，范围：最大位置限制到最小位置限制
* @author ACTION(migrate by LJ)
*/
void DJIBoard_PosCtrl(CAN_TypeDef *CANx, uint8_t ElmoNum, uint8_t posMode, int32_t pos);

/**
* @brief  电机电流控制
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  cur:电流大小，单位mA,范围：3508:-20A~20A , 2006:-10A~10A
* @author LJ
*/
void DJIBoard_CurCrl(CAN_TypeDef *CANx, uint8_t ElmoNum, int32_t cur);

/**
* @brief  配置加速度与减速度
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  acc：加速度，单位：脉冲每二次方秒
* @param  dec：减速度，单位：脉冲每二次方秒
* @author ACTION(migrate by LJ)
* @note
*/
void DJIBoard_SetAccAndDec(CAN_TypeDef *CANx, uint8_t ElmoNum, uint32_t acc, uint32_t dec);

/**
* @brief  配置位置环运行最大速度
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  vel: 速度，单位：脉冲每秒，范围：最小速度限制到最大速度限制
* @author ACTION(migrate by LJ)
* @note：速度正负号代表旋转的方向，大于零为正方向，小于零为负方向
*/
void DJIBoard_SetPosLoopVel(CAN_TypeDef *CANx, uint8_t ElmoNum, int32_t vel);

/**
* @brief  配置驱动器工作模式
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  unitMode：驱动器工作模式，范围：
			TORQUE_CONTROL_MODE：力矩控制模式，在该模式下可以执行TC电流命令
			SPEED_CONTROL_MODE：速度控制模式，在该模式下通过设置JV值控制速度
			MICRO_STEPPER_MODE：直流电机不能使用该模式
			DUAL_POSITION_MODE：双位置闭环模式
			SINGLE_POSITION_MODE：单位置闭环模式，在该模式下可以配置PA、PR、JV、PT或PVT运动
* @author ACTION(migrate by LJ)
* @note 只有在电机失能时可以配置该参数
*/
void DJIBoard_SetUnitMode(CAN_TypeDef *CANx, uint8_t ElmoNum, uint8_t unitMode);

/**
* @brief  配置运行速度
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  vel: 速度，单位：脉冲每秒，范围：最小速度限制到最大速度限制
* @author ACTION(migrate by LJ)
* @note：速度正负号代表旋转的方向，大于零为正方向，小于零为负方向
*/
void DJIBoard_SetJoggingVel(CAN_TypeDef *CANx, uint8_t ElmoNum, int32_t vel);

/**
* @brief  配置位置环命令
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  posMode: 位置环运行模式，范围：
				ABSOLUTE_MODE: 绝对位置模式
				RELATIVE_MODE: 相对位置模式
* @param  pos:位置命令，单位：脉冲，范围：最大位置限制到最小位置限制
* @author ACTION(migrate by LJ)
* @note：位置正负号代表旋转的方向，大于零为正方向，小于零为负方向
*/
void DJIBoard_SendPosCmd(CAN_TypeDef *CANx, uint8_t ElmoNum, uint8_t posMode, int32_t pos);

/**********************************特殊控制指令*******************************************/

/**
 * @brief 一条CAN消息同时控制四个电机速度环
 * 
 * @param vel 四个速度的数组
 * @author LJunius
 */
void DJIBoard_VelCtrlAll(int16_t vel[4]);

/**
 * @brief 一条CAN消息同时控制四个电机位置环
 * 
 * @param pos 四个位置的数组
 * @author LJunius
 */
void DJIBoard_PosCtrlAll(int16_t pos[4]);

/**********************************读取驱动器数据命令*************************************/

/**
* @brief  读取电机位置
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION(migrate by LJ)
 * @note：接收标识符为：0x00005850
*/
void DJIBoard_ReadActualPos(CAN_TypeDef *CANx, uint8_t ElmoNum);

/**
* @brief  读取电机速度
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION(migrate by LJ)
 * @note：接收标识符为：0x00005856
*/
void DJIBoard_ReadActualVel(CAN_TypeDef *CANx, uint8_t ElmoNum);

/** 
* @brief  读取电机电流
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION(migrate by LJ)
* @note：接收标识符为：0x00005858
*/
void DJIBoard_ReadActualCur(CAN_TypeDef *CANx, uint8_t ElmoNum);

void CAN_Callback_DJIBoard_ReadInfo(CAN_ConnMessage_s *data);
void CAN_Callback_DJIBoard_ReadAllPosInfo(CAN_ConnMessage_s *data);

#endif
