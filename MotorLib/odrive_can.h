#ifndef __ODRIVE_CAN_H
#define __ODRIVE_CAN_H

#include "motor_driver.h"
#ifdef USE_MTR_DRIVER_ODRIVE

#include "stm32f4xx.h"
#include "can_utils.h"

/* 宏和enum------------------------------------------------------------*/
typedef enum ODrive_Axis_e
{
    AXIS_0 = 0,
    AXIS_1 = 1
} ODrive_Axis_e;

// ODrive输入模式，是控制模式的高级应用 | 参考https://docs.odriverobotics.com/api/odrive.controller.inputmode
typedef enum ODrive_InputMode_e
{
    ODRIVE_INPUT_MODE_INACTIVE = 0, // 失能模式，期望值将保持为历史值
    ODRIVE_INPUT_MODE_PASSTHROUGH,  // 透传模式，子模式为ODrive_CtrlMode_e所列值
    ODRIVE_INPUT_MODE_VEL_RAMP,     // 速度爬升模式
    ODRIVE_INPUT_MODE_POS_FILTER,   // 位置滤波模式
    ODRIVE_INPUT_MODE_MIX_CHANNELS, // 官方还未实现，等待后续固件
    ODRIVE_INPUT_MODE_TRAP_TRAJ,    // 轨迹模式，可以调节油门曲线
    ODRIVE_INPUT_MODE_TORQUE_RAMP,  // 转矩爬升模式
    ODRIVE_INPUT_MODE_MIRROR,       // 镜像模式，期望值镜像为另一个电机(axis)的encoder estimates
    ODRIVE_INPUT_MODE_TUNING,       // 调试模式
} ODrive_InputMode_e;
/** @note inputmode和ctrlmode搭配使用，例如使用转速爬升模式时配合速度控制模式*/

// ODrive控制模式 | 参考https://docs.odriverobotics.com/api/odrive.controller.controlmode
typedef enum ODrive_CtrlMode_e
{
    ODRIVE_CTRL_MODE_VOLTAGE = 0, // 电压控制模式，等效于占空比模式
    ODRIVE_CTRL_MODE_TORQUE,      // 力矩控制模式，等效于电流环
    ODRIVE_CTRL_MODE_VEL,
    ODRIVE_CTRL_MODE_POS
} ODrive_CtrlMode_e;

// ODrive 状态 |  参考https://docs.odriverobotics.com/api/odrive.axis.axisstate
typedef enum ODrive_AxisState_e
{
    AXIS_STATE_UNDEFINED = 0,                          //<! will fall through to idle
    AXIS_STATE_IDLE = 1,                               //<! disable PWM and do nothing
    AXIS_STATE_STARTUP_SEQUENCE = 2,                   //<! the actual sequence is defined by the config.startup_... flags
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,          //<! run all calibration procedures, then idle
    AXIS_STATE_MOTOR_CALIBRATION = 4,                  //<! run motor calibration
    AXIS_STATE_SENSORLESS_CONTROL = 5,                 //<! run sensorless control
    AXIS_STATE_ENCODER_INDEX_SEARCH = 6,               //<! run encoder index search
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7,         //<! run encoder offset calibration
    AXIS_STATE_CLOSED_LOOP_CONTROL = 8,                //<! run closed loop control
    AXIS_STATE_LOCKIN_SPIN = 9,                        //<! run lockin spin
    AXIS_STATE_ENCODER_DIR_FIND = 10,                  //Run encoder direction search.
    AXIS_STATE_HOMING = 11,                            //<! run axis homing function
    AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION = 12, //Rotate the motor in lockin and calibrate hall polarity
    AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION = 13,    //Rotate the motor for 30s to calibrate hall sensor edge offsets
} ODrive_AxisState_e;

// ODrive CAN控制命令ID | CAN_ID = MTR_ID <<5 | CmdMsg
typedef enum ODrive_CAN_CmdMsg_e
{
    ODRIVE_MSG_CO_NMT_CTRL = 0x000, // CANOpen NMT Message REC
    ODRIVE_MSG_ODRIVE_HEARTBEAT,
    ODRIVE_MSG_ODRIVE_ESTOP,
    ODRIVE_MSG_GET_MOTOR_ERROR, // Errors
    ODRIVE_MSG_GET_ENCODER_ERROR,
    ODRIVE_MSG_GET_SENSORLESS_ERROR,
    ODRIVE_MSG_SET_AXIS_NODE_ID,
    ODRIVE_MSG_SET_AXIS_REQUESTED_STATE,
    ODRIVE_MSG_SET_AXIS_STARTUP_CONFIG,
    ODRIVE_MSG_GET_ENCODER_ESTIMATES,
    ODRIVE_MSG_GET_ENCODER_COUNT,
    ODRIVE_MSG_SET_CONTROLLER_MODES,
    ODRIVE_MSG_SET_INPUT_POS,
    ODRIVE_MSG_SET_INPUT_VEL,
    ODRIVE_MSG_SET_INPUT_CURRENT,
    ODRIVE_MSG_SET_VEL_LIMIT,
    ODRIVE_MSG_START_ANTICOGGING,
    ODRIVE_MSG_SET_TRAJ_VEL_LIMIT,
    ODRIVE_MSG_SET_TRAJ_ACCEL_LIMITS,
    ODRIVE_MSG_SET_TRAJ_A_PER_CSS,
    ODRIVE_MSG_GET_IQ,
    ODRIVE_MSG_GET_SENSORLESS_ESTIMATES,
    ODRIVE_MSG_RESET_ODRIVE,
    ODRIVE_MSG_GET_VBUS_VOLTAGE,
    ODRIVE_MSG_CLEAR_ERRORS,
    ODRIVE_MSG_CO_HEARTBEAT_CMD = 0x700, // CANOpen NMT Heartbeat  SEND
} ODrive_CAN_CmdMsg_e;

/* 结构体------------------------------------------------------------*/

// 从ODrive反馈得到的电机状态
typedef struct
{
    uint32_t axis_error;
    uint32_t axis_current_state;
    uint32_t motor_error;
    uint32_t encoder_error;
    uint32_t sensorless_error;
    float encoder_pos_estimate;
    float encoder_vel_estimate;
    int32_t encoder_shadow_count;
    int32_t encoder_cpr_count;
    float iq_setpoint;
    float iq_measured;
    float sensorless_pos_estimate;
    float sensorless_vel_estimate;
    float vbus_voltage;
} ODrive_AxisGetState_t;
/** @note 注意！ODrive的反馈机制是利用CAN报文的RTR模式，主控发送RTR_REMOTE型报文，ODrive返回相同ID的RTR_DATA型报文并将相应的反馈量装填在数据域*/

// ODrive期望状态数据结构
typedef struct
{
    uint16_t axis_node_id; // 注意！！标准帧ID范围0 to 63 (0x3F)
    uint32_t requested_state;
    int32_t control_mode;
    int32_t input_mode;
    float input_pos;
    float input_vel;
    float input_current;
    float input_torque;
    float vel_limit;
    float current_limit;
    float traj_vel_limit;
    float traj_accel_limit;
    float traj_decel_limit;
    float traj_a_per_css;
    int16_t vel_ff;
    int16_t torque_ff;
} ODrive_AxisSetState_t;

// ODrive电机类
typedef struct ODriveAxis_t
{
    uint16_t mtr_id;                 // 电机ID
    ODrive_AxisGetState_t get_state; // 期望状态
    ODrive_AxisSetState_t set_state; // 反馈状态
} ODriveAxis_t;

// ODrive驱动板类 | 包含两个电机成员(axis)
typedef struct ODrive_t
{
    ODriveAxis_t axis0;
    ODriveAxis_t axis1;
} ODrive_t;

/* 全局变量声明------------------------------------------------------------*/

extern ODrive_AxisGetState_t ODrive_CAN_RxBuff;

/* 函数声明------------------------------------------------------------*/

uint8_t ODrive_Init(ODrive_t *odrive);
uint8_t CAN_Callback_ODriveAxis(ODriveAxis_t *axis, CAN_ConnMessage_s *msg);
uint8_t ODrive_CANSendMsg(CAN_HandleTypeDef *hcan, ODrive_t *odrive, uint8_t axis, ODrive_CAN_CmdMsg_e msg);
// void odrive_can_update_config(ODrive_Axis_e axis, float pos);
// void odrive_can_enable_control(ODrive_Axis_e axis, ControlMode_t mode);
// void odrive_can_send_position(ODrive_Axis_e axis, float pos);

#endif
#endif /* __ODRIVE_CAN_H */