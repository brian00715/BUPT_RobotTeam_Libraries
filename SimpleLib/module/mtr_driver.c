/*******************************************************************************
 * Copyright:		BUPT
 * File Name:		mt_driver.c
 * Description:		driver 驱动实现
 * Author:			ZeroVoid
 * Version:			0.1
 * Data:			2019/10/22 Tue 19:01
 * Encoding:        UTF-8
 *******************************************************************************/
#include "mtr_driver.h"
#include "can_utils.h"

#ifdef EN_VESC_MOTOR_DRIVER
/**
 * @brief	VESC Can Command
 * @author  czh modified ZeroVoid ref Benjamin
 * @note    can ext sent modified by ZeroVoid
 */
void buffer_append_int16(uint8_t* buffer, int16_t number) {
    uint8_t index = 0;
	buffer[(index)++] = number >> 8;
	buffer[(index)++] = number;
}
void buffer_append_int32(uint8_t* buffer, int32_t number) {
    uint8_t index = 0;
	buffer[(index)++] = number >> 24;
	buffer[(index)++] = number >> 16;
	buffer[(index)++] = number >> 8;
	buffer[(index)++] = number;
}
void buffer_append_float16(uint8_t* buffer, float number, float scale) {
    buffer_append_int16(buffer, (int16_t)(number * scale));
}

void buffer_append_float32(uint8_t* buffer, float number, float scale) {
    buffer_append_int32(buffer, (int32_t)(number * scale));
}

/*czh add:设置对应id的占空比，已测试*/
void vesc_set_duty(uint8_t controller_id, float duty) {
    can_msg msg;
    msg.in[1] = 0;
	buffer_append_int32(msg.ui8, (int32_t)(duty * 1000.0));
	can_ext_send_msg(controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), &msg);
}

/*czh add:设置对应id的驱动电流，未测试*/
void vesc_set_current(uint8_t controller_id, float current) {
    can_msg msg = {0};
    buffer_append_int32(msg.ui8, (int32_t)(current * 1000.0));
	can_ext_send_msg(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), &msg);
}

/*czh add:设置对应id的刹车电流，未测试*/
void vesc_set_current_brake(uint8_t controller_id, float current) {
    can_msg msg = {0};
	buffer_append_int32(msg.ui8, (int32_t)(current * 1000.0));
	can_ext_send_msg(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), &msg);
}

/*czh add:设置对应id的转速rpm，未测试*/
void vesc_set_rpm(uint8_t controller_id, float rpm) {
    can_msg msg = {0};
	buffer_append_int32(msg.ui8, (int32_t)rpm);
	can_ext_send_msg(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), &msg);
}

/*czh add:设置对应id的位置，未测试*/
void vesc_set_pos(uint8_t controller_id, float pos) {
    can_msg msg = {0};
	buffer_append_int32(msg.ui8, (int32_t)(pos * 1000000.0));
	can_ext_send_msg(controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8), &msg);
}

/**
 * Set current relative to the minimum and maximum current limits.
 * czh add:设置最大/小的电流？ 未测试，作用未知
 * @param controller_id
 * The ID of the VESC to set the current on.
 *
 * @param current_rel
 * The relative current value, range [-1.0 1.0]
 */
void vesc_set_current_rel(uint8_t controller_id, float current_rel) {
    can_msg msg = {0};
	buffer_append_float32(msg.ui8, current_rel, 1e5);
	can_ext_send_msg(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8), &msg);
}

/**
 * Set brake current relative to the minimum current limit.
 * czh add:设置最小的刹车电流？ 未测试，作用未知
 * @param controller_id
 * The ID of the VESC to set the current on.
 *
 * @param current_rel
 * The relative current value, range [0.0 1.0]
 */
void vesc_set_current_brake_rel(uint8_t controller_id, float current_rel) {
    can_msg msg;
	buffer_append_float32(msg.ui8, current_rel, 1e5);
	can_ext_send_msg(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE_REL << 8), &msg);
}

/**
 * Set handbrake current.
 * czh add:设置手刹电流？ 未测试，上位机测试作用未知，慎用
 * @param controller_id
 * The ID of the VESC to set the handbrake current on.
 *
 * @param current_rel
 * The handbrake current value
 */
void vesc_set_handbrake(uint8_t controller_id, float current) {
    can_msg msg = {0};
	buffer_append_float32(msg.ui8, current, 1e3);
	can_ext_send_msg(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE << 8), &msg);
}

/**
 * Set handbrake current relative to the minimum current limit.
 * czh add:设置最小的手刹电流？ 未测试，作用未知
 * @param controller_id
 * The ID of the VESC to set the handbrake current on.
 *
 * @param current_rel
 * The relative handbrake current value, range [0.0 1.0]
 */
void vesc_set_handbrake_rel(uint8_t controller_id, float current_rel) {
    can_msg msg;
	buffer_append_float32(msg.ui8, current_rel, 1e5);
	can_ext_send_msg(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE_REL << 8), &msg);
}
#endif // EN_VESC_MOTOR_DRIVER

#ifdef EN_MOTOR_DRIVER
void md_set_duty(uint16_t id, int duty) {
    can_msg msg;
    msg.in[0] = MD_DUTY;
    msg.in[1] = duty;
    can_send_msg(id, &msg);
}

void md_set_speed(uint16_t id, int speed) {
    can_msg msg;
    msg.in[0] = MD_SPEED;
    msg.in[1] = speed;
    can_send_msg(id, &msg);
}

void md_set_position(uint16_t id, int position) {
    can_msg msg;
    msg.in[0] = MD_POSITION;
    msg.in[1] = position;
    can_send_msg(id, &msg);
}

#endif // EN_MOTRO