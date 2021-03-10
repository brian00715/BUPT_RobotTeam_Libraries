#include "cmd_func.h"
// TODO 分离ChassisLib和SimpleLib
// #include "base_chassis.h"
// #include "rm_cxxx_can.h"
// #include "rudder_chassis.h"
// #include "motor_driver.h"

// #define USE_CHASSIS_CMD

void CMD_Hello(int argc, char *argv[])
{
    uprintf("--Hello!\r\n");
}

//laser
#ifdef USE_LASER
void cmd_laser_print_diatance(int argc, char *argv[])
{
    laser_print_distance();
}

void CMD_Laser_SwitchPrintPos(int argc, char *argv[])
{
    Laser_PrintPos_Flag = atoi(argv[1]);
    if (Laser_PrintPos_Flag == 1)
    {
        uprintf("--CMD:Start print laser pos (%d)\r\n", Laser_PrintPos_Flag);
    }
    else
    {
        uprintf("--CMD:Stop print laser pos (%d)\r\n", Laser_PrintPos_Flag);
    }
    // laser_print_pos();
}

void CMD_Laser_SwitchPrintADCValue(int argc, char *argv[])
{
    Laser_PrintADCValue_Flag = atoi(argv[1]);
    if (Laser_PrintADCValue_Flag)
    {
        uprintf("--CMD:Start print laser adc value (%d)\r\n", Laser_PrintADCValue_Flag);
    }
    else
    {
        uprintf("--CMD:Stop print laser adc value (%d)\r\n", Laser_PrintADCValue_Flag);
    }
}
#endif // USE_LASER

/********************************[电机上位机控制]***********************************/
#ifdef USE_MOTORLIB
void CMD_VESC_SwitchPrintInfo(int argc, char *argv[])
{
    if (atoi(argv[1]) == 0)
    {
        VESC_SwitchPrintInfo_Flag = 0;
        uprintf("--VESC PrintInfo Closed!\r\n");
    }
    else if (atoi(argv[1]) == 1)
    {
        VESC_SwitchPrintInfo_Flag = 1;
        uprintf("--VESC PrintInfo Opened!\r\n");
    }
}
#ifdef USE_MTR_DRIVER_RM_CXXX
extern DJIMotor_t DJIMotor[4];
/*设置m2006的参数，用以手动控制*/
void CMD_M2006_SetCurrent(int argc, char *argv[])
{
    uint8_t index = atoi(argv[1]);
    DJIMotor[index].target_current = atoi(argv[2]);
    uprintf("M2006 current is %f.\r\n", atoi(argv[2]));
}
void CMD_Robomaster_SetRPM(int argc, char *argv[])
{
    RM_RPMControl_Flag = 1;
    RM_PosControl_Flag = 0;
    if (argc <= 4)
    {
        uprintf("##param num(4) error!##\r\n");
        return;
    }
    for (int i = 0; i < 4; i++)
    {
        Robomaster_RPMValue[i] = atoi(argv[i + 1]);
    }
    uprintf("--Robomaster speed set to: %d,%d,%d,%d.\r\n",
            Robomaster_RPMValue[0], Robomaster_RPMValue[1], Robomaster_RPMValue[2], Robomaster_RPMValue[3]);
}

void CMD_Robomaster_SetPosition(int argc, char *argv[])
{
    RM_RPMControl_Flag = 0;
    RM_PosControl_Flag = 1;
    if (argc <= 4)
    {
        uprintf("##param num(4) error!##\r\n");
        return;
    }
    for (int i = 0; i < 4; i++)
    {
        DJIMotor[i].target_position = (int)((atoi(argv[i + 1]) * 8192 / 360) + DJIMotor[i].total_angle);
    }
    uprintf("--Robomaster position set to: %d,%d,%d,%d.\r\n",
            DJIMotor[0].target_position, DJIMotor[1].target_position,
            DJIMotor[2].target_position, DJIMotor[3].target_position);
}

/*输入0关闭串口打印，输入1开启*/
void CMD_Robomaster_SwitchPrintInfo(int argc, char *argv[])
{
    if (atoi(argv[1]) == 0)
    {
        RM_PrintInfo_Flag = 0;
        uprintf("--Robomaster PrintInfo Closed!\r\n");
    }
    else if (atoi(argv[1]) == 1)
    {
        RM_PrintInfo_Flag = 1;
        uprintf("--Robomaster PrintInfo Opened!\r\n");
    }
}
#endif // DIRECT_CTRL_RM

void CMD_RMMtr_Ctrl(int argc, char *argv[])
{
    if (strcmp(argv[1], "motoron") == 0)
    {
        uprintf("motor%d is on\r\n", atoi(argv[2]));
        DJIBoard_MotorOn(CAN1, atoi(argv[2]));
    }
    else if (strcmp(argv[1], "motoroff") == 0)
    {
        uprintf("motor%d is off\r\n", atoi(argv[2]));
        DJIBoard_MotorOff(CAN1, atoi(argv[2]));
    }
    else if (strcmp(argv[1], "velcfg") == 0)
    {
        uprintf("motor%d's mode changes to velloopmode\r\n", atoi(argv[2]));
        DJIBoard_VelLoopCfg(CAN1, atoi(argv[2]), 0, 0);
    }
    else if (strcmp(argv[1], "poscfg") == 0)
    {
        uprintf("motor%d's mode changes to posLoopMode,maxv is %d\r\n", atoi(argv[2]), atoi(argv[3]));
        DJIBoard_PosLoopCfg(CAN1, atoi(argv[2]), 0, 0, atoi(argv[3]));
    }
    else if (strcmp(argv[1], "curcfg") == 0)
    {
        uprintf("motor%d's mode changes to curloopmode\r\n", atoi(argv[2]));
        DJIBoard_CurLoopCfg(CAN1, atoi(argv[2]));
    }
    else if (strcmp(argv[1], "velctr") == 0)
    {
        uprintf("motor%d's speed is set to %d\r\n", atoi(argv[2]), atoi(argv[3]));
        DJIBoard_VelCrl(CAN1, atoi(argv[2]), atoi(argv[3]));
    }
    else if (strcmp(argv[1], "posctr") == 0)
    {
        uprintf("motor%d's pos is set to %d\r\n", atoi(argv[2]), atoi(argv[3]));
        DJIBoard_PosCtrl(CAN1, atoi(argv[2]), ABSOLUTE_MODE, atoi(argv[3]));
    }
    else if (strcmp(argv[1], "curctr") == 0)
    {
        uprintf("motor%d's pos is set to %d\r\n", atoi(argv[2]), atoi(argv[3]));
        DJIBoard_CurCrl(CAN1, atoi(argv[2]), atoi(argv[3]));
    }
    else if (strcmp(argv[1], "qcur") == 0)
    {
        uprintf("query for motor%d's cur\r\n", atoi(argv[2]));
        DJIBoard_ReadActualCur(CAN1, atoi(argv[2]));
    }
    else if (strcmp(argv[1], "qpos") == 0) // 向东大驱动板发送查询CAN消息
    {
        uprintf("query for motor%d's pos\r\n", atoi(argv[2]));
        DJIBoard_ReadActualPos(CAN1, atoi(argv[2]));
    }
    else if (strcmp(argv[1], "qvel") == 0)
    {
        uprintf("query for motor%d's vel\r\n", atoi(argv[2]));
        DJIBoard_ReadActualVel(CAN1, atoi(argv[2]));
    }
    else if (strcmp(argv[1], "setallvel") == 0)
    {
        if (argc < 6)
        {
            uprintf("parameter error\r\n");
            return;
        }
        uprintf("set all vel");
        int16_t vel[4];
        for (int i = 0; i < 4; i++)
        {
            vel[i] = atoi(argv[2 + i]);
            uprintf("%d ", vel[i]);
        }
        uprintf("\r\n");
        DJIBoard_VelCtrlAll(vel);
    }
    else if (strcmp(argv[1], "setallpos") == 0)
    {
        if (argc < 6)
        {
            uprintf("parameter error\r\n");
            return;
        }
        uprintf("set all pos: ");
        int16_t pos[4];
        for (int i = 0; i < 4; i++)
        {
            pos[i] = (int16_t)atoi(argv[2 + i]);
            uprintf("%d ", pos[i]);
        }
        uprintf("\r\n");
        DJIBoard_PosCtrlAll(pos);
    }
    else if (strcmp(argv[1], "queryallpos") == 0)
    {
        uprintf("--motor1:%d motor2:%d motor3:%d motor4:%d\r\n",
                RM_MotorStatus[0].pos, RM_MotorStatus[1].pos,
                RM_MotorStatus[2].pos, RM_MotorStatus[3].pos);
    }
}
#endif
/********************************END***********************************/

/********************************[底盘控制]***********************************/
#ifdef USE_CHASSIS_CMD
extern float CMD_TargetSpeed;
extern float CMD_TargetDir;
extern float CMD_TargetOmega;
void CMD_Chassis_Teleop_Acc(int argc, char **argv)
{
    CMD_TargetSpeed += 0.05;
    uprintf("--Vel:%.2f, Dir:%.2f, Omega:%.2f\r\n", CMD_TargetSpeed, CMD_TargetDir, CMD_TargetOmega);
}

void CMD_Chassis_Teleop_Dec(int argc, char **argv)
{
    CMD_TargetSpeed -= 0.05;
    uprintf("--Vel:%.2f, Dir:%.2f, Omega:%.2f\r\n", CMD_TargetSpeed, CMD_TargetDir, CMD_TargetOmega);
}

void CMD_Chassis_Teleop_GoAhead(int argc, char **argv)
{
    CMD_TargetOmega = 0;
    CMD_TargetSpeed = 0.05;
    CMD_TargetDir = 1.5708;
    uprintf("--Vel:%.2f, Dir:%.2f, Omega:%.2f\r\n", CMD_TargetSpeed, CMD_TargetDir, CMD_TargetOmega);
}

void CMD_Chassis_Teleop_GoBack(int argc, char **argv)
{
    CMD_TargetOmega = 0;
    CMD_TargetSpeed = 0.05;
    CMD_TargetDir = -1.5708;
    uprintf("--Vel:%.2f, Dir:%.2f, Omega:%.2f\r\n", CMD_TargetSpeed, CMD_TargetDir, CMD_TargetOmega);
}

void CMD_Chassis_Teleop_TurnLeft(int argc, char **argv)
{
    CMD_TargetOmega += 0.5;
    uprintf("--Vel:%.2f, Dir:%.2f, Omega:%.2f\r\n", CMD_TargetSpeed, CMD_TargetDir, CMD_TargetOmega);
}

void CMD_Chassis_Teleop_TurnRight(int argc, char **argv)
{
    CMD_TargetOmega -= 0.5;
    uprintf("--Vel:%.2f, Dir:%.2f, Omega:%.2f\r\n", CMD_TargetSpeed, CMD_TargetDir, CMD_TargetOmega);
}

void CMD_Chassis_Teleop_Stop(int argc, char **argv)
{
    CMD_TargetSpeed = 0;
    CMD_TargetOmega = 0;
    uprintf("--Vel:%.2f, Dir:%.2f, Omega:%.2f\r\n", CMD_TargetSpeed, CMD_TargetDir, CMD_TargetOmega);
}

void CMD_Chassis_Teleop_ShiftRight(int argc, char **argv)
{
    CMD_TargetDir = 0;
    CMD_TargetSpeed = 0.05;
    uprintf("--Vel:%.2f, Dir:%.2f, Omega:%.2f\r\n", CMD_TargetSpeed, CMD_TargetDir, CMD_TargetOmega);
}
void CMD_Chassis_Teleop_ShiftLeft(int argc, char **argv)
{
    CMD_TargetDir = 3.14;
    CMD_TargetSpeed = 0.05;
    uprintf("--Vel:%.2f, Dir:%.2f, Omega:%.2f\r\n", CMD_TargetSpeed, CMD_TargetDir, CMD_TargetOmega);
}

/**
 * @brief 打印底盘信息
 * 
 * @note 配合全局变量PrintChassisStatus_Flag在main函数中调用
 */
void CMD_SwitchPrintChassisStatus(int argc, char *argv[])
{
    extern int PrintChassisStatus_Flag;
    PrintChassisStatus_Flag = (PrintChassisStatus_Flag + 1) % 2;
    if (PrintChassisStatus_Flag)
    {
        uprintf("--Start print chassis status!\r\n");
    }
    else
    {
        uprintf("--Stop print chassis status!\r\n");
    }
}

void CMD_SwitchPrintTargetStatus(int argc, char **argv)
{
    extern int PrintTargetStatus_Flag;
    PrintTargetStatus_Flag = (PrintTargetStatus_Flag + 1) % 2;
    if (PrintTargetStatus_Flag == 1)
    {
        uprintf("--Start print target status!\r\n");
    }
    else
    {
        uprintf("--Stop print target status!\r\n");
    }
}

void CMD_ChangePosMode(int argc, char **argv)
{
    BaseChassis.pos_mode = (BaseChassis.pos_mode + 1) % 2;
    if (BaseChassis.pos_mode == POS_MODE_ABSOLUTE)
    {
        uprintf("--CMD|Pos mode change to Absolute!\r\n");
    }
    else if (BaseChassis.pos_mode == POS_MODE_RELATIVE)
    {
        uprintf("--CMD|Pos mode change to Relative!\r\n");
    }
}

void CMD_ChangeCtrlMode(int argc, char **argv)
{
    BaseChassis.ctrl_mode = (BaseChassis.ctrl_mode + 1) % 5;
    RudderChassis.base->handbrake_flag = 0;
    uprintf("--CMD|Ctrl mode change to %d\r\n", BaseChassis.ctrl_mode);
}

extern float CMD_TargetYaw;
extern char YawTuning_Start;
void CMD_YawTuning(int argc, char **argv)
{
    BaseChassis.ctrl_mode = CTRL_MODE_TUNING;
    CMD_TargetYaw = __ANGLE2RAD(atof(argv[1]));
    extern PID_s YawPID;
    float kp = atof(argv[2]);
    float kd = atof(argv[3]);
    float ki = atof(argv[4]);
    float int_duty = atof(argv[5]);
    YawTuning_Start = atoi(argv[6]);
    YawPID.Kp = kp;
    YawPID.Kd = kd;
    YawPID.Ki = ki;
    YawPID.int_duty = int_duty;
    uprintf("--CMD|TargetYaw:%.2f kp:%.2f kd:%.2f ki:%.2f int_duty:%.2f start_flag:%d \r\n",
            CMD_TargetYaw, kp, kd, ki, int_duty, YawTuning_Start);
}

/**
 * @brief 设置偏航角PID的参数
 */
void CMD_SetYawPID(int argc, char **argv)
{
    extern PID_s YawPID;
    float kp = atof(argv[1]);
    float kd = atof(argv[2]);
    float ki = atof(argv[3]);
    float int_duty = atof(argv[4]);
    YawPID.Kp = kp;
    YawPID.Kd = kd;
    YawPID.Ki = ki;
    YawPID.int_duty = int_duty;
    uprintf("--CMD|YawPID - kp:%.2f kd:%.2f ki:%.2f int_duty:%.2f \r\n",
            kp, kd, ki, int_duty);
}

/**
 * @brief 设置法向修正向量的PID参数
 */
void CMD_SetNormalCorrPID(int argc, char **argv)
{
    extern PID_s NormalCorrPID_x, NormalCorrPID_y;
    float kp = atof(argv[1]);
    float kd = atof(argv[2]);
    float ki = atof(argv[3]);
    float int_duty = atof(argv[4]);
    NormalCorrPID_x.Kp = kp;
    NormalCorrPID_x.Kd = kd;
    NormalCorrPID_x.Ki = ki;
    NormalCorrPID_x.int_duty = int_duty;
    NormalCorrPID_y.Kp = kp;
    NormalCorrPID_y.Kd = kd;
    NormalCorrPID_y.Ki = ki;
    NormalCorrPID_y.int_duty = int_duty;
    uprintf("--CMD|NormalCorrPID - kp:%.2f kd:%.2f ki:%.2f int_duty:%.2f \r\n",
            kp, kd, ki, int_duty);
}

/**
 * @brief 设置原地锁定PID参数 
 */
void CMD_SetLockPID(int argc, char **argv)
{
    extern PID_s LockPID;
    float kp = atof(argv[1]);
    float kd = atof(argv[2]);
    float ki = atof(argv[3]);
    LockPID.Kp = kp;
    LockPID.Kd = kd;
    LockPID.Ki = ki;
    uprintf("--CMD|LockPID - kp:%.2f kd:%.2f ki:%.2f \r\n",
            kp, kd, ki);
}

void CMD_SwitchPrintMotorStatus(int argc, char **argv)
{
    extern uint8_t SW_PrintMotorStatus_Flag;
    SW_PrintMotorStatus_Flag = (SW_PrintMotorStatus_Flag + 1) % 2;
    if (SW_PrintMotorStatus_Flag == 1)
    {
        uprintf("--Start print motor status!\r\n");
    }
    else
    {
        uprintf("--Stop print motor status!\r\n");
    }
}

#endif
/********************************END***********************************/

void CMD_FuncInit(void)
{
    CMD_Add("hello", "", CMD_Hello);

#ifdef USE_MOTORLIB
    //motor
    CMD_Add("vesc_switch_print_info", "0 to close;1 to open", CMD_VESC_SwitchPrintInfo);
    CMD_Add("rm", "", CMD_RMMtr_Ctrl);
    CMD_Add("SwitchPrintMotorStatus", "press to change ", CMD_SwitchPrintMotorStatus);
#endif
#ifdef USE_CHASSIS_CMD
    //chassis
    CMD_Add("ChangePosMode", "", CMD_ChangePosMode);
    CMD_Add("ChangeCtrlMode", "", CMD_ChangeCtrlMode);
    CMD_Add("SwitchPrintTargetStatus", "press to change ", CMD_SwitchPrintTargetStatus);
    CMD_Add("SwitchPrintChassisStatus", "", CMD_SwitchPrintChassisStatus);
    CMD_Add("YawTuning", "<TargetYaw><kp><kd><ki><intTime><StartFlag>", CMD_YawTuning);
    CMD_Add("SetYawPID", "<kp><kd><ki><intTime>", CMD_SetYawPID);
    CMD_Add("SetNormalCorrPID", "<kp><kd><ki><intTime>", CMD_SetNormalCorrPID);
    CMD_Add("SetDriveMotorCurr", "", CMD_SetDriveMotorsCurr);
    CMD_Add("SetDriveMotorDuty", "", CMD_SetDriveMotorsDuty);
    CMD_Add("SetTargetPoint", "<x><y>", CMD_Chassis_SetTargetPoint);
    CMD_Add("SetLockPID", "", CMD_SetLockPID);

    // teleop
    CMD_Add("Teleop_Acc", "", CMD_Chassis_Teleop_Acc);
    CMD_Add("Teleop_Dec", "", CMD_Chassis_Teleop_Dec);
    CMD_Add("Teleop_GoAhead", "", CMD_Chassis_Teleop_GoAhead);
    CMD_Add("Teleop_GoBack", "", CMD_Chassis_Teleop_GoBack);
    CMD_Add("Teleop_TurnLeft", "", CMD_Chassis_Teleop_TurnLeft);
    CMD_Add("Teleop_TurnRight", "", CMD_Chassis_Teleop_TurnRight);
    CMD_Add("Teleop_ShiftLeft", "", CMD_Chassis_Teleop_ShiftLeft);
    CMD_Add("Teleop_ShiftRight", "", CMD_Chassis_Teleop_ShiftRight);
    CMD_Add("Teleop_Stop", "", CMD_Chassis_Teleop_Stop);
#endif

#ifdef USE_LASER
    CMD_Add("laser_print_diatance", "", cmd_laser_print_diatance);
    CMD_Add("laser_switch_print_pos", "", CMD_Laser_SwitchPrintPos);
    CMD_Add("laser_switch_print_adc_value", "", CMD_Laser_SwitchPrintADCValue);
#endif
}