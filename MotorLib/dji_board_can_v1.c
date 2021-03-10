/**
 * @file dji_ctr.c
 * @author ActionLab & LJenius
 * @brief 大疆驱动板v1.0（基于STM32标准库） CAN通信协议相关
 * @version 0.1
 * @date 2021-07-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "motor_driver.h"
#ifdef USE_MTR_DRIVER_DJI_BOARD_V1

#include "dji_board_can_v1.h"
#include "utils.h"
#include "simplelib.h"

MotorType Motor[8] = {0};
RM_MotorStatus_t RM_MotorStatus[4]; // 四个大疆电机的状态结构体

/**
* @brief  电机使能（通电）
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION(migrate by LJ)
* @note ELMO驱动器默认初始状态为电机失能，使用电机时需要对其进行使能
*       部分驱动器参数需要在电机失能状态下才可以配置
*/
void DJIBoard_MotorOn(CAN_TypeDef *CANx, uint8_t ElmoNum)
{
    //第一个数发送Motor控制命令(和)，第二个数发送1给电机使能(通电)
    uint32_t data[1][2] = {
        0x00004F4D,
        0x00000001,
    };
    CAN_Message_u msg;
    uint32_t StdId = ELMO_DEVICE_BASEID + ElmoNum;

    msg.ui8[0] = *(unsigned long *)&data[0][0] & 0xff;
    msg.ui8[1] = (*(unsigned long *)&data[0][0] >> 8) & 0xff;
    msg.ui8[2] = (*(unsigned long *)&data[0][0] >> 16) & 0xff;
    msg.ui8[3] = (*(unsigned long *)&data[0][0] >> 24) & 0xff;
    msg.ui8[4] = *(unsigned long *)&data[0][1] & 0xff;
    msg.ui8[5] = (*(unsigned long *)&data[0][1] >> 8) & 0xff;
    msg.ui8[6] = (*(unsigned long *)&data[0][1] >> 16) & 0xff;
    msg.ui8[7] = (*(unsigned long *)&data[0][1] >> 24) & 0xff;

    CAN_SendStdMsg(StdId, &msg);
}

/**
* @brief  电机失能（断电）
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION(migrate by LJ)
*/
void DJIBoard_MotorOff(CAN_TypeDef *CANx, uint8_t ElmoNum)
{
    uint32_t data[1][2] = {
        0x00004F4D, 0x00000000, //MO  0
    };
    CAN_Message_u msg;
    uint32_t StdId = ELMO_DEVICE_BASEID + ElmoNum;

    msg.ui8[0] = *(unsigned long *)&data[0][0] & 0xff;
    msg.ui8[1] = (*(unsigned long *)&data[0][0] >> 8) & 0xff;
    msg.ui8[2] = (*(unsigned long *)&data[0][0] >> 16) & 0xff;
    msg.ui8[3] = (*(unsigned long *)&data[0][0] >> 24) & 0xff;
    msg.ui8[4] = *(unsigned long *)&data[0][1] & 0xff;
    msg.ui8[5] = (*(unsigned long *)&data[0][1] >> 8) & 0xff;
    msg.ui8[6] = (*(unsigned long *)&data[0][1] >> 16) & 0xff;
    msg.ui8[7] = (*(unsigned long *)&data[0][1] >> 24) & 0xff;

    CAN_SendStdMsg(StdId, &msg);
}

/**
* @brief  驱动器速度环初始化
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  acc：加速度，单位：脉冲每二次方秒
* @param  dec：减速度，单位：脉冲每二次方秒
* @author ACTION(migrate by LJ)
* @note 在速度环初始化后才可以使能电机！！
*/
void DJIBoard_VelLoopCfg(CAN_TypeDef *CANx, uint8_t ElmoNum, uint32_t acc, uint32_t dec)
{
    DJIBoard_SetUnitMode(CANx, ElmoNum, SPEED_CONTROL_MODE);

    DJIBoard_SetAccAndDec(CANx, ElmoNum, acc, dec);
}

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
void DJIBoard_PosLoopCfg(CAN_TypeDef *CANx, uint8_t ElmoNum, uint32_t acc, uint32_t dec, uint32_t vel)
{
    DJIBoard_SetUnitMode(CANx, ElmoNum, POSITION_CONTROL_MODE);
    DJIBoard_SetPosLoopVel(CANx, ElmoNum, vel);
}

/**
* @brief  驱动器电流环初始化
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author LJ
* @note 在电流环初始化后才可以使能电机！！
*/
void DJIBoard_CurLoopCfg(CAN_TypeDef *CANx, uint8_t ElmoNum)
{
    DJIBoard_SetUnitMode(CANx, ElmoNum, CUR_CONTROL_MODE);
}

/**
* @brief  电机速度控制
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  vel: 速度，单位：脉冲每秒，范围：最小速度限制到最大速度限制
* @author ACTION(migrate by LJ)
*/
void DJIBoard_VelCrl(CAN_TypeDef *CANx, uint8_t ElmoNum, int32_t vel)
{
    DJIBoard_SetJoggingVel(CANx, ElmoNum, vel);
}

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
void DJIBoard_PosCtrl(CAN_TypeDef *CANx, uint8_t ElmoNum, uint8_t posMode, int32_t pos)
{
    DJIBoard_SendPosCmd(CANx, ElmoNum, posMode, pos);
}

/**
* @brief  电机电流控制
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  cur:电流大小，单位mA,范围：3508:-20A~20A , 2006:-10A~10A
* @author LJ
*/
void DJIBoard_CurCrl(CAN_TypeDef *CANx, uint8_t ElmoNum, int32_t cur)
{
    uint32_t data[1][2] = {
        0x00004250, 0x00000000, //PA
    };
    data[0][1] = cur;

    CAN_Message_u msg;
    uint32_t StdId = ELMO_DEVICE_BASEID + ElmoNum;

    msg.ui8[0] = *(unsigned long *)&data[0][0] & 0xff;
    msg.ui8[1] = (*(unsigned long *)&data[0][0] >> 8) & 0xff;
    msg.ui8[2] = (*(unsigned long *)&data[0][0] >> 16) & 0xff;
    msg.ui8[3] = (*(unsigned long *)&data[0][0] >> 24) & 0xff;
    msg.ui8[4] = *(unsigned long *)&data[0][1] & 0xff;
    msg.ui8[5] = (*(unsigned long *)&data[0][1] >> 8) & 0xff;
    msg.ui8[6] = (*(unsigned long *)&data[0][1] >> 16) & 0xff;
    msg.ui8[7] = (*(unsigned long *)&data[0][1] >> 24) & 0xff;

    CAN_SendStdMsg(StdId, &msg);
}
/**
* @brief  配置加速度与减速度
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  acc：加速度，单位：脉冲每二次方秒
* @param  dec：减速度，单位：脉冲每二次方秒
* @author ACTION(migrate by LJ)
* @note
*/
void DJIBoard_SetAccAndDec(CAN_TypeDef *CANx, uint8_t ElmoNum, uint32_t acc, uint32_t dec)
{
    //第一个数据发送AC\DC命令，第二个数据发送命令值
    uint32_t data[2][2] = {
        0x00004341, 0x00000000, //AC
        0x00004344, 0x00000000  //DC
    };
    CAN_Message_u msg;

    uint32_t StdId = ELMO_DEVICE_BASEID + ElmoNum;

    data[0][1] = acc;
    data[1][1] = dec;

    for (uint8_t i = 0; i < 2; i++)
    {
        msg.ui8[0] = *(unsigned long *)&data[i][0] & 0xff;
        msg.ui8[1] = (*(unsigned long *)&data[i][0] >> 8) & 0xff;
        msg.ui8[2] = (*(unsigned long *)&data[i][0] >> 16) & 0xff;
        msg.ui8[3] = (*(unsigned long *)&data[i][0] >> 24) & 0xff;
        msg.ui8[4] = *(unsigned long *)&data[i][1] & 0xff;
        msg.ui8[5] = (*(unsigned long *)&data[i][1] >> 8) & 0xff;
        msg.ui8[6] = (*(unsigned long *)&data[i][1] >> 16) & 0xff;
        msg.ui8[7] = (*(unsigned long *)&data[i][1] >> 24) & 0xff;

        CAN_SendStdMsg(StdId, &msg);
    }
}

/**
* @brief  配置位置环运行最大速度
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  vel: 速度，单位：脉冲每秒，范围：最小速度限制到最大速度限制
* @author ACTION(migrate by LJ)
* @note：速度正负号代表旋转的方向，大于零为正方向，小于零为负方向
*/
void DJIBoard_SetPosLoopVel(CAN_TypeDef *CANx, uint8_t ElmoNum, int32_t vel)
{
    //第一个数据发送SP命令，第二个数据发送命令值
    uint32_t data[1][2] = {
        0x00005053, 0x00000000, //SP
    };
    data[0][1] = vel;

    CAN_Message_u msg;
    uint32_t StdId = ELMO_DEVICE_BASEID + ElmoNum;

    msg.ui8[0] = *(unsigned long *)&data[0][0] & 0xff;
    msg.ui8[0] = *(unsigned long *)&data[0][0] & 0xff;
    msg.ui8[1] = (*(unsigned long *)&data[0][0] >> 8) & 0xff;
    msg.ui8[2] = (*(unsigned long *)&data[0][0] >> 16) & 0xff;
    msg.ui8[3] = (*(unsigned long *)&data[0][0] >> 24) & 0xff;
    msg.ui8[4] = *(unsigned long *)&data[0][1] & 0xff;
    msg.ui8[5] = (*(unsigned long *)&data[0][1] >> 8) & 0xff;
    msg.ui8[6] = (*(unsigned long *)&data[0][1] >> 16) & 0xff;
    msg.ui8[7] = (*(unsigned long *)&data[0][1] >> 24) & 0xff;

    CAN_SendStdMsg(StdId, &msg);
}

/**
* @brief  配置驱动器工作模式
* @param  CANx 所使用的CAN通道编号
* @param  ElmoNum 驱动器ID号，范围：0~128，0为广播用ID号
* @param  unitMode 驱动器工作模式，范围
*   @arg TORQUE_CONTROL_MODE 力矩控制模式，在该模式下可以执行TC电流命令
*	@arg SPEED_CONTROL_MODE 速度控制模式，在该模式下通过设置JV值控制速度
*	@arg ICRO_STEPPER_MODE 直流电机不能使用该模式
*	@arg DUAL_POSITION_MODE 双位置闭环模式
*	@arg INGLE_POSITION_MODE 单位置闭环模式，在该模式下可以配置PA、PR、JV、PT或PVT运动
* @author ACTION(migrate by LJ)
* @warning 只有在电机失能时可以配置该参数
*/
void DJIBoard_SetUnitMode(CAN_TypeDef *CANx, uint8_t ElmoNum, uint8_t unitMode)
{
    //第一个数据发送UM命令，第二个数据发送模式
    uint32_t data[1][2] = {
        0x00004D55, 0x00000000, //UM
    };
    data[0][1] = unitMode;

    CAN_Message_u msg;
    uint32_t StdId = ELMO_DEVICE_BASEID + ElmoNum;

    msg.ui8[0] = *(unsigned long *)&data[0][0] & 0xff;
    msg.ui8[1] = (*(unsigned long *)&data[0][0] >> 8) & 0xff;
    msg.ui8[2] = (*(unsigned long *)&data[0][0] >> 16) & 0xff;
    msg.ui8[3] = (*(unsigned long *)&data[0][0] >> 24) & 0xff;
    msg.ui8[4] = *(unsigned long *)&data[0][1] & 0xff;
    msg.ui8[5] = (*(unsigned long *)&data[0][1] >> 8) & 0xff;
    msg.ui8[6] = (*(unsigned long *)&data[0][1] >> 16) & 0xff;
    msg.ui8[7] = (*(unsigned long *)&data[0][1] >> 24) & 0xff;

    CAN_SendStdMsg(StdId, &msg);
}

/**
* @brief  配置运行速度
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  vel: 速度，单位：脉冲每秒，范围：最小速度限制到最大速度限制
* @author ACTION(migrate by LJ)
* @note：速度正负号代表旋转的方向，大于零为正方向，小于零为负方向
*/
void DJIBoard_SetJoggingVel(CAN_TypeDef *CANx, uint8_t ElmoNum, int32_t vel)
{
    //第一个数据发送JV命令，第二个数据发送命令值
    uint32_t data[1][2] = {
        0x0000564A, 0x00000000, //JV
    };
    data[0][1] = vel;

    CAN_Message_u msg;
    uint32_t StdId = ELMO_DEVICE_BASEID + ElmoNum;

    msg.ui8[0] = *(unsigned long *)&data[0][0] & 0xff;
    msg.ui8[1] = (*(unsigned long *)&data[0][0] >> 8) & 0xff;
    msg.ui8[2] = (*(unsigned long *)&data[0][0] >> 16) & 0xff;
    msg.ui8[3] = (*(unsigned long *)&data[0][0] >> 24) & 0xff;
    msg.ui8[4] = *(unsigned long *)&data[0][1] & 0xff;
    msg.ui8[5] = (*(unsigned long *)&data[0][1] >> 8) & 0xff;
    msg.ui8[6] = (*(unsigned long *)&data[0][1] >> 16) & 0xff;
    msg.ui8[7] = (*(unsigned long *)&data[0][1] >> 24) & 0xff;

    CAN_SendStdMsg(StdId, &msg);
}

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
void DJIBoard_SendPosCmd(CAN_TypeDef *CANx, uint8_t ElmoNum, uint8_t posMode, int32_t pos)
{
    uint32_t data[1][2] = {
        0x00000000, 0x00000000, //PA
    };
    if (posMode == ABSOLUTE_MODE)
    {
        data[0][0] = 0x00004150; //绝对
    }
    data[0][1] = pos;

    CAN_Message_u msg;
    uint32_t StdId = ELMO_DEVICE_BASEID + ElmoNum;

    msg.ui8[0] = *(unsigned long *)&data[0][0] & 0xff;
    msg.ui8[1] = (*(unsigned long *)&data[0][0] >> 8) & 0xff;
    msg.ui8[2] = (*(unsigned long *)&data[0][0] >> 16) & 0xff;
    msg.ui8[3] = (*(unsigned long *)&data[0][0] >> 24) & 0xff;
    msg.ui8[4] = *(unsigned long *)&data[0][1] & 0xff;
    msg.ui8[5] = (*(unsigned long *)&data[0][1] >> 8) & 0xff;
    msg.ui8[6] = (*(unsigned long *)&data[0][1] >> 16) & 0xff;
    msg.ui8[7] = (*(unsigned long *)&data[0][1] >> 24) & 0xff;

    CAN_SendStdMsg(StdId, &msg);
}

/**********************************特殊控制指令*******************************************/

/**
 * @brief 一条CAN消息同时控制四个电机速度环
 * @param vel 四个速度的数组
 * @author LJunius
 */
void DJIBoard_VelCtrlAll(int16_t vel[4])
{
    CAN_Message_u msg;
    uint32_t StdId = ELMO_DEVICE_BASEID - 1;
    for (int i = 0; i < 4; i++)
    {
        msg.ui8[i * 2] = vel[i] & 0xff;
        msg.ui8[i * 2 + 1] = (vel[i] >> 8) & 0xff;
    }
    CAN_SendStdMsg(StdId, &msg);
}

/**
 * @brief 一条CAN消息同时控制四个电机位置环
 * 
 * @param pos 四个位置的数组
 * @author LJunius
 */
void DJIBoard_PosCtrlAll(int16_t pos[4])
{
    CAN_Message_u msg;
    uint32_t StdId = ELMO_DEVICE_BASEID - 2;
    for (int i = 0; i < 4; i++)
    {
        msg.ui8[i * 2] = pos[i] & 0xff;
        msg.ui8[i * 2 + 1] = (pos[i] >> 8) & 0xff;
    }
    CAN_SendStdMsg(StdId, &msg);
}
/**********************************读取驱动器数据命令*************************************/

/**
* @brief  读取电机位置
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION(migrate by LJ)
 * @note：接收标识符为：0x00005850
*/
void DJIBoard_ReadActualPos(CAN_TypeDef *CANx, uint8_t ElmoNum)
{
    uint32_t data[1][2] = {
        0x40005850, 0x00000000, //PX
    };
    CAN_Message_u msg;
    uint32_t StdId = ELMO_DEVICE_BASEID + ElmoNum;

    msg.ui8[0] = *(unsigned long *)&data[0][0] & 0xff;
    msg.ui8[1] = (*(unsigned long *)&data[0][0] >> 8) & 0xff;
    msg.ui8[2] = (*(unsigned long *)&data[0][0] >> 16) & 0xff;
    msg.ui8[3] = (*(unsigned long *)&data[0][0] >> 24) & 0xff;
    msg.ui8[4] = *(unsigned long *)&data[0][1] & 0xff;
    msg.ui8[5] = (*(unsigned long *)&data[0][1] >> 8) & 0xff;
    msg.ui8[6] = (*(unsigned long *)&data[0][1] >> 16) & 0xff;
    msg.ui8[7] = (*(unsigned long *)&data[0][1] >> 24) & 0xff;

    CAN_SendStdMsg(StdId, &msg);
}

/**
* @brief  读取电机速度
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
 * @note：接收标识符为：0x00005856
*/
void DJIBoard_ReadActualVel(CAN_TypeDef *CANx, uint8_t ElmoNum)
{
    uint32_t data[1][2] = {
        0x40005856, 0x00000000, //VX
    };
    CAN_Message_u msg;
    uint32_t StdId = ELMO_DEVICE_BASEID + ElmoNum;

    msg.ui8[0] = *(unsigned long *)&data[0][0] & 0xff;
    msg.ui8[1] = (*(unsigned long *)&data[0][0] >> 8) & 0xff;
    msg.ui8[2] = (*(unsigned long *)&data[0][0] >> 16) & 0xff;
    msg.ui8[3] = (*(unsigned long *)&data[0][0] >> 24) & 0xff;
    msg.ui8[4] = *(unsigned long *)&data[0][1] & 0xff;
    msg.ui8[5] = (*(unsigned long *)&data[0][1] >> 8) & 0xff;
    msg.ui8[6] = (*(unsigned long *)&data[0][1] >> 16) & 0xff;
    msg.ui8[7] = (*(unsigned long *)&data[0][1] >> 24) & 0xff;

    CAN_SendStdMsg(StdId, &msg);
}
/** 
* @brief  读取电机电流
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author LJ
* @note：接收标识符为：0x00005149
*/
void DJIBoard_ReadActualCur(CAN_TypeDef *CANx, uint8_t ElmoNum)
{
    uint32_t data[1][2] = {
        0x40005149, 0x00000000, //VX
    };
    CAN_Message_u msg;
    uint32_t StdId = ELMO_DEVICE_BASEID + ElmoNum;

    msg.ui8[0] = *(unsigned long *)&data[0][0] & 0xff;
    msg.ui8[1] = (*(unsigned long *)&data[0][0] >> 8) & 0xff;
    msg.ui8[2] = (*(unsigned long *)&data[0][0] >> 16) & 0xff;
    msg.ui8[3] = (*(unsigned long *)&data[0][0] >> 24) & 0xff;
    msg.ui8[4] = *(unsigned long *)&data[0][1] & 0xff;
    msg.ui8[5] = (*(unsigned long *)&data[0][1] >> 8) & 0xff;
    msg.ui8[6] = (*(unsigned long *)&data[0][1] >> 16) & 0xff;
    msg.ui8[7] = (*(unsigned long *)&data[0][1] >> 24) & 0xff;

    CAN_SendStdMsg(StdId, &msg);
}

void CAN_Callback_DJIBoard_ReadInfo(CAN_ConnMessage_s *data)
{
    int id = (data->payload.in[0]) >> 16;
    int index = ((data->payload.in[0]) << 16) >> 16;

    if (index == 0x00005850)
    {
        //位置信息
        int num = data->payload.in[1];
        RM_MotorStatus[id].pos = num;
        uprintf("motor %d's pos =%d\r\n", id, num);
    }
    else if (index == 0x00005856)
    {
        //速度信息
        int num = data->payload.in[1];
        RM_MotorStatus[id].rpm = (float)num;
        uprintf("motor %d's vel =%d\r\n", id, num / 1000);
    }
    else if (index == 0x00005149)
    {
        //电流信息
        float num = data->payload.fl[1];
        RM_MotorStatus[id].current = (float)num;
        uprintf("motor %d's cur =%f\r\n", id, num);
    }
}

void CAN_Callback_DJIBoard_ReadAllPosInfo(CAN_ConnMessage_s *data)
{
    // uprintf("--Received DJI CAN Msg\r\n");
    // 直接得到的是电机输出轴的位置（已经除过819.2了）
    RM_MotorStatus[0].pos = (float)__ANGLE2RAD(data->payload.i16[0]);
    RM_MotorStatus[1].pos = (float)__ANGLE2RAD(data->payload.i16[1]);
    RM_MotorStatus[2].pos = (float)__ANGLE2RAD(data->payload.i16[2]);
    RM_MotorStatus[3].pos = (float)__ANGLE2RAD(data->payload.i16[3]);
}

#endif