#include "cmd_func.h"
#include "chassis_common.h"
#include "robomaster.h"

void CMD_Hello(int argc, char *argv[])
{
    uprintf("--Hello!\r\n");
}

//laser
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

//point
void CMD_Point_PrintPath(int argc, char *argv[])
{
    point_print_path();
}

/********************************[电机上位机控制]***********************************/
extern DJIMotor_t DJIMotor[4];
/*设置m2006的参数，用以手动控制*/
void CMD_M2006_SetCurrent(int argc, char *argv[])
{
    uint8_t index = atoi(argv[1]);
    DJIMotor[index].target_current = atoi(argv[2]);
    uprintf("M2006 current is %f.\r\n", atoi(argv[2]));
}

/*设置本杰明电调的参数,用以手动控制*/
void CMD_VESC_SetParam(int argc, char *argv[])
{
    vesc.mode = atoi(argv[1]);
    switch (vesc.mode)
    {
    case 0:
    {
        vesc.target_duty = atof(argv[2]);
        uprintf("--Vesc work in Duty at %f\r\n", atof(argv[2]));
        break;
    }
    case 1:
    {
        vesc.target_current = atof(argv[2]);
        uprintf("--Vesc work in Current at %f\r\n", atof(argv[2]));
        break;
    }
    case 2:
    {
        vesc.target_rpm = atof(argv[2]);
        uprintf("--Vesc work in RPM at %f\r\n", atof(argv[2]));
        break;
    }
    case 3:
    {
        vesc.target_position = atof(argv[2]);
        uprintf("--Vesc work in Position at %f\r\n", atof(argv[2]));
        break;
    }
    default:
    {
        uprintf("--parameter explain:\r\n ");
        uprintf("  0:duty\r\n1:current\r\n2:speed\r\n3:position\r\n");
        break;
    }
    }
}

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

void CMD_Robomater_StopByAngle(int argc, char *argv[])
{
    Robomaster_OpenAngleControl_Flag = 1;
    Robomaster_TargetOffsetAngle = (uint32_t)atoi(argv[1]);
    // MoterDriver_M2006_Current = atof(argv[2]);
    uprintf("--robomaster motor will stop when rotated by %d degrees.\r\n", Robomaster_TargetOffsetAngle);
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

void cmd_robomaster_control(int argc, char *argv[])
{
    if (strcmp(argv[1], "motoron") == 0)
    {
        uprintf("motor%d is on\r\n", atoi(argv[2]));
        MotorOn(CAN1, atoi(argv[2]));
    }
    else if (strcmp(argv[1], "motoroff") == 0)
    {
        uprintf("motor%d is off\r\n", atoi(argv[2]));
        MotorOff(CAN1, atoi(argv[2]));
    }
    else if (strcmp(argv[1], "velcfg") == 0)
    {
        uprintf("motor%d's mode changes to velloopmode\r\n", atoi(argv[2]));
        VelLoopCfg(CAN1, atoi(argv[2]), 0, 0);
    }
    else if (strcmp(argv[1], "poscfg") == 0)
    {
        uprintf("motor%d's mode changes to posLoopMode,maxv is %d\r\n", atoi(argv[2]), atoi(argv[3]));
        PosLoopCfg(CAN1, atoi(argv[2]), 0, 0, atoi(argv[3]));
    }
    else if (strcmp(argv[1], "curcfg") == 0)
    {
        uprintf("motor%d's mode changes to curloopmode\r\n", atoi(argv[2]));
        CurLoopCfg(CAN1, atoi(argv[2]));
    }
    else if (strcmp(argv[1], "velctr") == 0)
    {
        uprintf("motor%d's speed is set to %d\r\n", atoi(argv[2]), atoi(argv[3]));
        DJI_VelCrl(CAN1, atoi(argv[2]), atoi(argv[3]));
    }
    else if (strcmp(argv[1], "posctr") == 0)
    {
        uprintf("motor%d's pos is set to %d\r\n", atoi(argv[2]), atoi(argv[3]));
        DJI_PosCtrl(CAN1, atoi(argv[2]), ABSOLUTE_MODE, atoi(argv[3]));
    }
    else if (strcmp(argv[1], "curctr") == 0)
    {
        uprintf("motor%d's pos is set to %d\r\n", atoi(argv[2]), atoi(argv[3]));
        CurCrl(CAN1, atoi(argv[2]), atoi(argv[3]));
    }
    else if (strcmp(argv[1], "qcur") == 0)
    {
        uprintf("query for motor%d's cur\r\n", atoi(argv[2]));
        ReadActualCur(CAN1, atoi(argv[2]));
    }
    else if (strcmp(argv[1], "qpos") == 0) // 向东大驱动板发送查询CAN消息
    {
        uprintf("query for motor%d's pos\r\n", atoi(argv[2]));
        DJI_ReadActualPos(CAN1, atoi(argv[2]));
    }
    else if (strcmp(argv[1], "qvel") == 0)
    {
        uprintf("query for motor%d's vel\r\n", atoi(argv[2]));
        ReadActualVel(CAN1, atoi(argv[2]));
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
        DJI_velCtrAll(vel);
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
        DJI_posCtrlAll(pos);
    }
    else if (strcmp(argv[1], "queryallpos") == 0)
    {
        uprintf("--motor1:%d motor2:%d motor3:%d motor4:%d\r\n",
                RM_MotorStatus[0].pos, RM_MotorStatus[1].pos,
                RM_MotorStatus[2].pos, RM_MotorStatus[3].pos);
    }
}

/********************************END***********************************/

int16_t Chassis_MoterDuty[3];
/********************************[底盘控制]***********************************/
void CMD_Chassis_Move(int argc, char *argv[])
{
    // test_value[0] = atof(argv[1]);
    // test_value[1] = atof(argv[2]);
    // test_value[2] = atof(argv[3]);
    Chassis_MoterDuty[0] = atoi(argv[1]);
    Chassis_MoterDuty[1] = atoi(argv[2]);
    Chassis_MoterDuty[2] = atoi(argv[3]);
    uprintf("move in duty of %d  %d %d\r\n", Chassis_MoterDuty[0],
            Chassis_MoterDuty[1], Chassis_MoterDuty[2]);
}

extern float CMD_ChassisSpeed;
extern float CMD_ChassisDir;
extern float CMD_ChassisOmega;
void CMD_Chassis_Teleop_GoAhead(int argc, char **argv)
{
    CMD_ChassisSpeed += 0.1;
    uprintf("--Vel:%.2f, Dir:%.2f, Omega:%.2f\r\n", CMD_ChassisSpeed, CMD_ChassisDir, CMD_ChassisOmega);
}

void CMD_Chassis_Teleop_GoBack(int argc, char **argv)
{
    CMD_ChassisSpeed -= 0.1;
    uprintf("--Vel:%.2f, Dir:%.2f, Omega:%.2f\r\n", CMD_ChassisSpeed, CMD_ChassisDir, CMD_ChassisOmega);
}

void CMD_Chassis_Teleop_TurnLeft(int argc, char **argv)
{
    CMD_ChassisDir += 5;
    uprintf("--Vel:%.2f, Dir:%.2f, Omega:%.2f\r\n", CMD_ChassisSpeed, CMD_ChassisDir, CMD_ChassisOmega);
}

void CMD_Chassis_Teleop_TurnRight(int argc, char **argv)
{
    CMD_ChassisDir -= 5;
    uprintf("--Vel:%.2f, Dir:%.2f, Omega:%.2f\r\n", CMD_ChassisSpeed, CMD_ChassisDir, CMD_ChassisOmega);
}

void CMD_Chassis_Teleop_Stop(int argc, char **argv)
{
    CMD_ChassisSpeed = 0;
    uprintf("--Vel:%.2f, Dir:%.2f, Omega:%.2f\r\n", CMD_ChassisSpeed, CMD_ChassisDir, CMD_ChassisOmega);
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


void CMD_SW_ChangePosMode(int argc, char **argv)
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

float CMD_TargetYaw = 0;
extern int YawTuning_Start;
void CMD_YawTuning(int argc, char **argv)
{
    BaseChassis.ctrl_mode = CTRL_MODE_TUNING;
    CMD_TargetYaw = ANGLE2RAD(atof(argv[1]));
    extern PID_t YawPID;
    float kp = atof(argv[2]);
    float kd = atof(argv[3]);
    float ki = atof(argv[4]);
    float int_duty = atof(argv[5]);
    YawTuning_Start = atoi(argv[6]);
    YawPID.KP = kp;
    YawPID.KD = kd;
    YawPID.KI = ki;
    YawPID.I_TIME = int_duty;
    uprintf("--CMD|TargetYaw:%.2f kp:%.2f kd:%.2f ki:%.2f int_duty:%.2f start_flag:%d \r\n", CMD_TargetYaw, kp, kd, ki, int_duty, YawTuning_Start);
}

/**
 * @brief 设置偏航角PID的参数
 */
void CMD_SetYawPID(int argc, char **argv)
{
    extern PID_t YawPID;
    float kp = atof(argv[1]);
    float kd = atof(argv[2]);
    float ki = atof(argv[3]);
    float int_duty = atof(argv[4]);
    YawPID.KP = kp;
    YawPID.KD = kd;
    YawPID.KI = ki;
    YawPID.I_TIME = int_duty;
    uprintf("--CMD|YawPID - kp:%.2f kd:%.2f ki:%.2f int_duty:%.2f \r\n",
            kp, kd, ki, int_duty);
}

/**
 * @brief 设置法向修正向量的PID参数
 */
void CMD_SetNormalCorrPID(int argc, char **argv)
{
    extern PID_t NormalCorrPID;
    float kp = atof(argv[1]);
    float kd = atof(argv[2]);
    float ki = atof(argv[3]);
    float int_duty = atof(argv[4]);
    NormalCorrPID.KP = kp;
    NormalCorrPID.KD = kd;
    NormalCorrPID.KI = ki;
    NormalCorrPID.I_TIME = int_duty;
    uprintf("--CMD|NormalCorrPID - kp:%.2f kd:%.2f ki:%.2f int_duty:%.2f \r\n",
            kp, kd, ki, int_duty);
}

/********************************END***********************************/

void cmd_func_init(void)
{
    cmd_add("hello", "", CMD_Hello);
    //point
    cmd_add("point_print_path", "", CMD_Point_PrintPath);

    //motor
    cmd_add("vesc", "<mode(0,1,2)> <value>", CMD_VESC_SetParam);
    cmd_add("vesc_switch_print_info", "0 to close;1 to open", CMD_VESC_SwitchPrintInfo);
    cmd_add("rm", "", cmd_robomaster_control);

    //chassis
    cmd_add("SW_ChangePosMode", "", CMD_SW_ChangePosMode);
    cmd_add("SwitchPrintTargetStatus", "press to change ", CMD_SwitchPrintTargetStatus);
    cmd_add("SwitchPrintChassisStatus", "", CMD_SwitchPrintChassisStatus);
    cmd_add("YawTuning", "<TargetYaw><kp><kd><ki><intTime><StartFlag>", CMD_YawTuning);
    cmd_add("SetYawPID", "<kp><kd><ki><intTime>", CMD_SetYawPID);
    cmd_add("SetNormalCorrPID", "<kp><kd><ki><intTime>", CMD_SetNormalCorrPID);

    // teleop
    cmd_add("SW_Teleop_GoAhead", "", CMD_Chassis_Teleop_GoAhead);
    cmd_add("SW_Teleop_GoBack", "", CMD_Chassis_Teleop_GoBack);
    cmd_add("SW_Teleop_TurnLeft", "", CMD_Chassis_Teleop_TurnLeft);
    cmd_add("SW_Teleop_TurnRight", "", CMD_Chassis_Teleop_TurnRight);
    cmd_add("SW_Teleop_Stop", "", CMD_Chassis_Teleop_Stop);

    //laser
    cmd_add("laser_print_diatance", "", cmd_laser_print_diatance);
    cmd_add("laser_switch_print_pos", "", CMD_Laser_SwitchPrintPos);
    cmd_add("laser_switch_print_adc_value", "", CMD_Laser_SwitchPrintADCValue);
}