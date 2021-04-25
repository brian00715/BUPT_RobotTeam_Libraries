/*******************************************************************************
 * Copyright:		BUPT
 * File Name:		mtr_driver.h
 * Description:		Motor Driver 通信封装
 * Author:			ZeroVoid
 * Version:			0.1
 * Data:			2019/10/22 Tue 13:53
 * Encoding:		UTF-8
 *******************************************************************************/
#ifndef __MTR_DRIVER_H
#define __MTR_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "simplelib_cfg.h"

#ifdef SL_MOTOR_DRIVER
#include <stdint.h>

/*******************************************************************************
 * Benjamin Motor Driver CAN Communication
 *******************************************************************************/
#ifdef EN_VESC_MOTOR_DRIVER
/**
 * @brief	VESC CAN Command
 * @author	czh ref benjamin
 */
typedef enum {
	CAN_PACKET_SET_DUTY = 0,
	CAN_PACKET_SET_CURRENT,
	CAN_PACKET_SET_CURRENT_BRAKE,
	CAN_PACKET_SET_RPM,
	CAN_PACKET_SET_POS,
	CAN_PACKET_FILL_RX_BUFFER,
	CAN_PACKET_FILL_RX_BUFFER_LONG,
	CAN_PACKET_PROCESS_RX_BUFFER,
	CAN_PACKET_PROCESS_SHORT_BUFFER,
	CAN_PACKET_STATUS,
	CAN_PACKET_SET_CURRENT_REL,
	CAN_PACKET_SET_CURRENT_BRAKE_REL,
	CAN_PACKET_SET_CURRENT_HANDBRAKE,
	CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
	CAN_PACKET_STATUS_2,
	CAN_PACKET_STATUS_3,
	CAN_PACKET_STATUS_4,
	CAN_PACKET_PING,
	CAN_PACKET_PONG,
	CAN_PACKET_DETECT_APPLY_ALL_FOC,
	CAN_PACKET_DETECT_APPLY_ALL_FOC_RES,
	CAN_PACKET_CONF_CURRENT_LIMITS,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS,
	CAN_PACKET_CONF_CURRENT_LIMITS_IN,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN,
	CAN_PACKET_CONF_FOC_ERPMS,
	CAN_PACKET_CONF_STORE_FOC_ERPMS,
	CAN_PACKET_STATUS_5
} CAN_PACKET_ID;

void vesc_set_duty(uint8_t controller_id, float duty);
void vesc_set_current(uint8_t controller_id, float current);
void vesc_set_current_brake(uint8_t controller_id, float current);
void vesc_set_rpm(uint8_t controller_id, float rpm);
void vesc_set_pos(uint8_t controller_id, float pos);
void vesc_set_current_rel(uint8_t controller_id, float current_rel);
void vesc_set_current_brake_rel(uint8_t controller_id, float current_rel);
void vesc_set_handbrake(uint8_t controller_id, float current);
void vesc_set_handbrake_rel(uint8_t controller_id, float current_rel);

#endif // EN_VESC_MOTOR_DRIVER

/*******************************************************************************
 * Normal Motor Driver CAN Communication
 *******************************************************************************/
#ifdef EN_MOTOR_DRIVER
/**
 * @brief	给HX 驱动卡&DriverPro 代码的CAN发送通信协议封装
 * @note	can_msg.in[0] 取值含义.
 */
typedef enum normal_driver {
    MD_DUTY = 0,    // can_msg.in[1] 取值范围-100~100
    MD_SPEED = 1,
    MD_POSITION = 2
} MD_DRIVER_ENUM;

/**
 * @brief	接收 HX 驱动卡&DriverPro 代码CAN消息协议封装
 * @note	index 0: speed; index 1: position. All float
 */
typedef enum normal_driver_packet {
	MDP_SPEED,	// Motor Driver Packet
	MDP_POSITION
} MD_PCK;	// Motor Driver Packet

/**
 * @brief	HX 驱动卡状态存储结构体
 * @note	保存speed&position数据
 */
struct md_state {
	float speed;
	float position;
};

/**
 * @brief	Normal Motor Driver Functions
 */
void md_set_duty(uint16_t id, int duty);
void md_set_speed(uint16_t id, int speed);
void md_set_position(uint16_t id, int position);

#endif // EN_MOTOR_DRIVER

#endif // SL_MOTOR_DRIVER
#ifdef __cplusplus
}
#endif

#endif /* __MTR_DRIVER_H */