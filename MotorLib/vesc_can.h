/**
  ******************************************************************************
  * @file           vesc_can.h
  * @author         ZohyCao
  * @version        1.0
  * @brief          本杰明电调(VESC) CAN总线驱动程序
  *                 适用于本杰明电调（HW:410\412\420, FW:3.61\3.62）
  * 使用步骤：
  *     1.使用前需修改comm_can_transmit_eid()函数，使用自己的CAN拓展帧函数发送指令
  * 
  * 注意事项：
  *     1.与大疆robomaster的电调一起使用时可能会造成指令冲突，使用时应测试ID是否合理
  * 
  * TODO:
  *     1.增加CAN PING函数
  *         
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 ZohyCao
  *
  ******************************************************************************
  */
#ifndef VESC_CAN_H_
#define VESC_CAN_H_

/* Includes ---------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Public Macro---------------------------------------------------*/
//settings
#define VESC_CAN_ENABLE 1

/* Public Types---------------------------------------------------*/
typedef uint32_t systime_t; //为了与本杰明电调代码保持一致

/* Datatypes---------------------------------------------------*/
typedef struct
{
	int id;
	float rpm;
	float current;
	float duty;
	float position;
} VESC;

//CAN commands
typedef enum
{
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

// CAN status modes
typedef enum
{
	CAN_STATUS_DISABLED = 0,
	CAN_STATUS_1,
	CAN_STATUS_1_2,
	CAN_STATUS_1_2_3,
	CAN_STATUS_1_2_3_4,
	CAN_STATUS_1_2_3_4_5
} CAN_STATUS_MODE;

typedef struct
{
	int id;
	systime_t rx_time;
	float rpm;
	float current;
	float duty;
} can_status_msg;

typedef struct
{
	int id;
	systime_t rx_time;
	float amp_hours;
	float amp_hours_charged;
} can_status_msg_2;

typedef struct
{
	int id;
	systime_t rx_time;
	float watt_hours;
	float watt_hours_charged;
} can_status_msg_3;

typedef struct
{
	int id;
	systime_t rx_time;
	float temp_fet;
	float temp_motor;
	float current_in;
	float pid_pos_now;
} can_status_msg_4;

typedef struct
{
	int id;
	systime_t rx_time;
	float v_in;
	int32_t tacho_value;
} can_status_msg_5;

/* Buffer Functions ---------------------------------------------------*/
void buffer_append_int16(uint8_t *buffer, int16_t number, int32_t *index);
void buffer_append_uint16(uint8_t *buffer, uint16_t number, int32_t *index);
void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index);
void buffer_append_uint32(uint8_t *buffer, uint32_t number, int32_t *index);
void buffer_append_float16(uint8_t *buffer, float number, float scale, int32_t *index);
void buffer_append_float32(uint8_t *buffer, float number, float scale, int32_t *index);
void buffer_append_float32_auto(uint8_t *buffer, float number, int32_t *index);
int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index);
uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index);
int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index);
uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index);
float buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index);
float buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index);
float buffer_get_float32_auto(const uint8_t *buffer, int32_t *index);

/* CAN Functions ---------------------------------------------------*/
//常用
void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len);	   //拓展帧发送函数
void comm_can_transmit_sid(uint32_t id, uint8_t *data, uint8_t len);		   //标准帧发送函数
void comm_can_set_duty(uint8_t controller_id, float duty);					   //设置对应ID占空比
void comm_can_set_current(uint8_t controller_id, float current);			   //设置对应ID电流环
void comm_can_set_current_brake(uint8_t controller_id, float current);		   //设置对应ID刹车电流
void comm_can_set_rpm(uint8_t controller_id, float rpm);					   //设置对应ID转速环
void comm_can_set_pos(uint8_t controller_id, float pos);					   //设置对应ID位置环
void comm_can_set_current_rel(uint8_t controller_id, float current_rel);	   //设置电流限幅
void comm_can_set_current_brake_rel(uint8_t controller_id, float current_rel); //设置刹车电流限幅
void comm_can_set_handbrake(uint8_t controller_id, float current);			   //设置刹车电流
void comm_can_set_handbrake_rel(uint8_t controller_id, float current_rel);	   //设置刹车电流限幅

//CAN指令命令解析函数，只用于参考解析过程，不可直接调用
void CAN_Analysis_Reference(void);

//高级功能，没什么用
void comm_can_conf_current_limits(uint8_t controller_id,
								  bool store, float min, float max);
void comm_can_conf_current_limits_in(uint8_t controller_id,
									 bool store, float min, float max);
void comm_can_conf_foc_erpms(uint8_t controller_id,
							 bool store, float foc_openloop_rpm, float foc_sl_erpm);
can_status_msg *comm_can_get_status_msg_index(int index);
can_status_msg *comm_can_get_status_msg_id(int id);
can_status_msg_2 *comm_can_get_status_msg_2_index(int index);
can_status_msg_2 *comm_can_get_status_msg_2_id(int id);
can_status_msg_3 *comm_can_get_status_msg_3_index(int index);
can_status_msg_3 *comm_can_get_status_msg_3_id(int id);
can_status_msg_4 *comm_can_get_status_msg_4_index(int index);
can_status_msg_4 *comm_can_get_status_msg_4_id(int id);
can_status_msg_5 *comm_can_get_status_msg_5_index(int index);
can_status_msg_5 *comm_can_get_status_msg_5_id(int id);

#endif /* VESC_CAN_H_ */
