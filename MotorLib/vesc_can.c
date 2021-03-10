/**
  ******************************************************************************
  * @file           vesc_can.c
  * @author         Simon
  * @version        1.0
  * @brief          本杰明电调(VESC) CAN总线驱动程序
  *                 适用于本杰明电调（HW:410\412\420, FW:3.61\3.62）
  * 使用步骤：
  *     1.使用前需修改comm_can_transmit_eid()函数，使用自己的CAN拓展帧函数发送指令
  * 
  * 注意事项：
  *     1.与大疆robomaster的电调一起使用时可能会造成指令冲突，使用时应测试ID是否合理
  *         
  */
#include "vesc_can.h"
#include "motor_driver.h"
#ifdef USE_MTR_DRIVER_VESC

/* Includes ---------------------------------------------------*/
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "can_utils.h"
#include "main.h"

/* CAN Functions---------------------------------------------------*/

int VESC_StatusBag_Flag = 0;
int VESC_SwitchPrintInfo_Flag = 0;
/**
 * @brief 本杰明电调反馈状态包解析函数
 **/
void CAN_Callback_VESC(MotorDriver_s *vesc, CAN_Message_u *data)
{
  int32_t index = 0;
  vesc->now_position = buffer_get_int16(data->ui8, &index) / 50;
  vesc->now_rpm = buffer_get_int32(data->ui8, &index);
}

void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len)
{
    if (len > 8)
    {
        len = 8;
    }

#if VESC_CAN_ENABLE
    CAN_SendExtMsg(id, (CAN_Message_u *)data); //须改为自己的can发送函数，拓展帧
#else
    (void)id;
    (void)data;
    (void)len;
#endif
}

void comm_can_transmit_sid(uint32_t id, uint8_t *data, uint8_t len)
{
    if (len > 8)
    {
        len = 8;
    }

#if VESC_CAN_ENABLE
    CAN_SendStdMsg((uint16_t)id, (CAN_Message_u *)data); //须改为自己的can发送函数，标准帧
#else
    (void)id;
    (void)data;
    (void)len;
#endif
}

/**
 * @brief 设置驱动器目标占空比 
 * 
 * @param controller_id 
 * @param duty 0.24=24%
 */
void comm_can_set_duty(uint8_t controller_id, float duty)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
    comm_can_transmit_eid(controller_id |
                              ((uint32_t)CAN_PACKET_SET_DUTY << 8),
                          buffer, send_index);
}

/**
 * @brief 设置本杰明电调输出电流
 * @param id 电调接收can id
 * @param current 电流值/A
 **/
void comm_can_set_current(uint8_t controller_id, float current)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
    comm_can_transmit_eid(controller_id |
                              ((uint32_t)CAN_PACKET_SET_CURRENT << 8),
                          buffer, send_index);
}

void comm_can_set_current_brake(uint8_t controller_id, float current)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
    comm_can_transmit_eid(controller_id |
                              ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8),
                          buffer, send_index);
}

void comm_can_set_rpm(uint8_t controller_id, float rpm)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)rpm, &send_index);
    comm_can_transmit_eid(controller_id |
                              ((uint32_t)CAN_PACKET_SET_RPM << 8),
                          buffer, send_index);
}

void comm_can_set_pos(uint8_t controller_id, float pos)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(pos * 1000000.0), &send_index);
    comm_can_transmit_eid(controller_id |
                              ((uint32_t)CAN_PACKET_SET_POS << 8),
                          buffer, send_index);
}

/**
 * Set current relative to the minimum and maximum current limits.
 *
 * @param controller_id
 * The ID of the VESC to set the current on.
 *
 * @param current_rel
 * The relative current value, range [-1.0 1.0]
 */
void comm_can_set_current_rel(uint8_t controller_id, float current_rel)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_float32(buffer, current_rel, 1e5, &send_index);
    comm_can_transmit_eid(controller_id |
                              ((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8),
                          buffer, send_index);
}

/**
 * Set brake current relative to the minimum current limit.
 *
 * @param controller_id
 * The ID of the VESC to set the current on.
 *
 * @param current_rel
 * The relative current value, range [0.0 1.0]
 */
void comm_can_set_current_brake_rel(uint8_t controller_id, float current_rel)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_float32(buffer, current_rel, 1e5, &send_index);
    comm_can_transmit_eid(controller_id |
                              ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE_REL << 8),
                          buffer, send_index);
}

/**
 * Set handbrake current.
 *
 * @param controller_id
 * The ID of the VESC to set the handbrake current on.
 *
 * @param current_rel
 * The handbrake current value
 */
void comm_can_set_handbrake(uint8_t controller_id, float current)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_float32(buffer, current, 1e3, &send_index);
    comm_can_transmit_eid(controller_id |
                              ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE << 8),
                          buffer, send_index);
}

/**
 * Set handbrake current relative to the minimum current limit.
 *
 * @param controller_id
 * The ID of the VESC to set the handbrake current on.
 *
 * @param current_rel
 * The relative handbrake current value, range [0.0 1.0]
 */
void comm_can_set_handbrake_rel(uint8_t controller_id, float current_rel)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_float32(buffer, current_rel, 1e5, &send_index);
    comm_can_transmit_eid(controller_id |
                              ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE_REL << 8),
                          buffer, send_index);
}

/**
 * Update current limits on VESC on CAN-bus.
 *
 * @param controller_id
 * ID of the VESC.
 *
 * @param store
 * Store parameters in emulated EEPROM (FLASH).
 *
 * @param min
 * Minimum current (negative value).
 *
 * @param max
 * Maximum current.
 */
void comm_can_conf_current_limits(uint8_t controller_id,
                                  bool store, float min, float max)
{
    int32_t send_index = 0;
    uint8_t buffer[8];
    buffer_append_float32(buffer, min, 1e3, &send_index);
    buffer_append_float32(buffer, max, 1e3, &send_index);
    comm_can_transmit_eid(controller_id |
                              ((uint32_t)(store ? CAN_PACKET_CONF_STORE_CURRENT_LIMITS : CAN_PACKET_CONF_CURRENT_LIMITS) << 8),
                          buffer, send_index);
}

/**
 * Update input current limits on VESC on CAN-bus.
 *
 * @param controller_id
 * ID of the VESC.
 *
 * @param store
 * Store parameters in emulated EEPROM (FLASH).
 *
 * @param min
 * Minimum current (negative value).
 *
 * @param max
 * Maximum current.
 */
void comm_can_conf_current_limits_in(uint8_t controller_id,
                                     bool store, float min, float max)
{
    int32_t send_index = 0;
    uint8_t buffer[8];
    buffer_append_float32(buffer, min, 1e3, &send_index);
    buffer_append_float32(buffer, max, 1e3, &send_index);
    comm_can_transmit_eid(controller_id |
                              ((uint32_t)(store ? CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN : CAN_PACKET_CONF_CURRENT_LIMITS_IN) << 8),
                          buffer, send_index);
}

/* Buffer Functions---------------------------------------------------*/
// 用以将can的8位数据段逐位填充
void buffer_append_int16(uint8_t *buffer, int16_t number, int32_t *index)
{
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void buffer_append_uint16(uint8_t *buffer, uint16_t number, int32_t *index)
{
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index)
{
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void buffer_append_uint32(uint8_t *buffer, uint32_t number, int32_t *index)
{
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void buffer_append_float16(uint8_t *buffer, float number, float scale, int32_t *index)
{
    buffer_append_int16(buffer, (int16_t)(number * scale), index);
}

void buffer_append_float32(uint8_t *buffer, float number, float scale, int32_t *index)
{
    buffer_append_int32(buffer, (int32_t)(number * scale), index);
}

void buffer_append_float32_auto(uint8_t *buffer, float number, int32_t *index)
{
    int e = 0;
    float sig = frexpf(number, &e);
    float sig_abs = fabsf(sig);
    uint32_t sig_i = 0;

    if (sig_abs >= 0.5)
    {
        sig_i = (uint32_t)((sig_abs - 0.5f) * 2.0f * 8388608.0f);
        e += 126;
    }

    uint32_t res = ((e & 0xFF) << 23) | (sig_i & 0x7FFFFF);
    if (sig < 0)
    {
        res |= 1U << 31;
    }

    buffer_append_uint32(buffer, res, index);
}

int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index)
{
    int16_t res = ((uint16_t)buffer[*index]) << 8 |
                  ((uint16_t)buffer[*index + 1]);
    *index += 2;
    return res;
}

uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index)
{
    uint16_t res = ((uint16_t)buffer[*index]) << 8 |
                   ((uint16_t)buffer[*index + 1]);
    *index += 2;
    return res;
}

int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index)
{
    int32_t res = ((uint32_t)buffer[*index]) << 24 |
                  ((uint32_t)buffer[*index + 1]) << 16 |
                  ((uint32_t)buffer[*index + 2]) << 8 |
                  ((uint32_t)buffer[*index + 3]);
    *index += 4;
    return res;
}

uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index)
{
    uint32_t res = ((uint32_t)buffer[*index]) << 24 |
                   ((uint32_t)buffer[*index + 1]) << 16 |
                   ((uint32_t)buffer[*index + 2]) << 8 |
                   ((uint32_t)buffer[*index + 3]);
    *index += 4;
    return res;
}

float buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index)
{
    return (float)buffer_get_int16(buffer, index) / scale;
}

float buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index)
{
    return (float)buffer_get_int32(buffer, index) / scale;
}

float buffer_get_float32_auto(const uint8_t *buffer, int32_t *index)
{
    uint32_t res = buffer_get_uint32(buffer, index);

    int e = (res >> 23) & 0xFF;
    uint32_t sig_i = res & 0x7FFFFF;
    bool neg = res & (1U << 31);

    float sig = 0.0;
    if (e != 0 || sig_i != 0)
    {
        sig = (float)sig_i / (8388608.0 * 2.0) + 0.5;
        e -= 126;
    }

    if (neg)
    {
        sig = -sig;
    }

    return ldexpf(sig, e);
}
/* Buffer Functions [END]---------------------------------------------------*/

//
///**
// * Update FOC ERPM settings on VESC on CAN-bus.
// *
// * @param controller_id
// * ID of the VESC.
// *
// * @param store
// * Store parameters in emulated EEPROM (FLASH).
// *
// * @param foc_openloop_rpm
// * Run in openloop below this ERPM in sensorless mode.
// *
// * @param foc_sl_erpm
// * Use sensors below this ERPM in sensored mode.
// */
//void comm_can_conf_foc_erpms(uint8_t controller_id,
//                             bool store, float foc_openloop_rpm, float foc_sl_erpm)
//{
//    int32_t send_index = 0;
//    uint8_t buffer[8];
//    buffer_append_float32(buffer, foc_openloop_rpm, 1e3, &send_index);
//    buffer_append_float32(buffer, foc_sl_erpm, 1e3, &send_index);
//    comm_can_transmit_eid(controller_id |
//                              ((uint32_t)(store ? CAN_PACKET_CONF_STORE_FOC_ERPMS : CAN_PACKET_CONF_FOC_ERPMS) << 8),
//                          buffer, send_index);
//}
//
///**
// * Get status message by index.
// *
// * @param index
// * Index in the array
// *
// * @return
// * The message or 0 for an invalid index.
// */
//can_status_msg *comm_can_get_status_msg_index(int index)
//{
//    if (index < CAN_STATUS_MSGS_TO_STORE)
//    {
//        return &stat_msgs[index];
//    }
//    else
//    {
//        return 0;
//    }
//}
//
///**
// * Get status message by id.
// *
// * @param id
// * Id of the controller that sent the status message.
// *
// * @return
// * The message or 0 for an invalid id.
// */
//can_status_msg *comm_can_get_status_msg_id(int id)
//{
//    for (int i = 0; i < CAN_STATUS_MSGS_TO_STORE; i++)
//    {
//        if (stat_msgs[i].id == id)
//        {
//            return &stat_msgs[i];
//        }
//    }
//
//    return 0;
//}
//
///**
// * Get status message 2 by index.
// *
// * @param index
// * Index in the array
// *
// * @return
// * The message or 0 for an invalid index.
// */
//can_status_msg_2 *comm_can_get_status_msg_2_index(int index)
//{
//    if (index < CAN_STATUS_MSGS_TO_STORE)
//    {
//        return &stat_msgs_2[index];
//    }
//    else
//    {
//        return 0;
//    }
//}
//
///**
// * Get status message 2 by id.
// *
// * @param id
// * Id of the controller that sent the status message.
// *
// * @return
// * The message or 0 for an invalid id.
// */
//can_status_msg_2 *comm_can_get_status_msg_2_id(int id)
//{
//    for (int i = 0; i < CAN_STATUS_MSGS_TO_STORE; i++)
//    {
//        if (stat_msgs_2[i].id == id)
//        {
//            return &stat_msgs_2[i];
//        }
//    }
//
//    return 0;
//}
//
///**
// * Get status message 3 by index.
// *
// * @param index
// * Index in the array
// *
// * @return
// * The message or 0 for an invalid index.
// */
//can_status_msg_3 *comm_can_get_status_msg_3_index(int index)
//{
//    if (index < CAN_STATUS_MSGS_TO_STORE)
//    {
//        return &stat_msgs_3[index];
//    }
//    else
//    {
//        return 0;
//    }
//}
//
///**
// * Get status message 3 by id.
// *
// * @param id
// * Id of the controller that sent the status message.
// *
// * @return
// * The message or 0 for an invalid id.
// */
//can_status_msg_3 *comm_can_get_status_msg_3_id(int id)
//{
//    for (int i = 0; i < CAN_STATUS_MSGS_TO_STORE; i++)
//    {
//        if (stat_msgs_3[i].id == id)
//        {
//            return &stat_msgs_3[i];
//        }
//    }
//
//    return 0;
//}
//
///**
// * Get status message 4 by index.
// *
// * @param index
// * Index in the array
// *
// * @return
// * The message or 0 for an invalid index.
// */
//can_status_msg_4 *comm_can_get_status_msg_4_index(int index)
//{
//    if (index < CAN_STATUS_MSGS_TO_STORE)
//    {
//        return &stat_msgs_4[index];
//    }
//    else
//    {
//        return 0;
//    }
//}
//
///**
// * Get status message 4 by id.
// *
// * @param id
// * Id of the controller that sent the status message.
// *
// * @return
// * The message or 0 for an invalid id.
// */
//can_status_msg_4 *comm_can_get_status_msg_4_id(int id)
//{
//    for (int i = 0; i < CAN_STATUS_MSGS_TO_STORE; i++)
//    {
//        if (stat_msgs_4[i].id == id)
//        {
//            return &stat_msgs_4[i];
//        }
//    }
//
//    return 0;
//}
//
///**
// * Get status message 5 by index.
// *
// * @param index
// * Index in the array
// *
// * @return
// * The message or 0 for an invalid index.
// */
//can_status_msg_5 *comm_can_get_status_msg_5_index(int index)
//{
//    if (index < CAN_STATUS_MSGS_TO_STORE)
//    {
//        return &stat_msgs_5[index];
//    }
//    else
//    {
//        return 0;
//    }
//}
//
///**
// * Get status message 5 by id.
// *
// * @param id
// * Id of the controller that sent the status message.
// *
// * @return
// * The message or 0 for an invalid id.
// */
//can_status_msg_5 *comm_can_get_status_msg_5_id(int id)
//{
//    for (int i = 0; i < CAN_STATUS_MSGS_TO_STORE; i++)
//    {
//        if (stat_msgs_5[i].id == id)
//        {
//            return &stat_msgs_5[i];
//        }
//    }
//
//    return 0;
//}
//
//
////CAN指令命令解析函数，只用于参考解析过程，不可直接调用
//void CAN_Analysis_Reference(void)
//{
//    return;
//    //具体解析过程见下
//#if 0
//    if (rxmsg.IDE == CAN_IDE_EXT)
//    {
//        uint8_t id = rxmsg.EID & 0xFF;
//        VESC_CAN_PACKET_ID cmd = rxmsg.EID >> 8;
//        if (id == 255 || id == controller_id)
//        {
//            switch (cmd)
//            {
//            case CAN_PACKET_SET_DUTY:
//                ind = 0;
//                mc_interface_set_duty(buffer_get_float32(rxmsg.data8, 1e5, &ind));
//                timeout_reset();
//                break;
//
//            case CAN_PACKET_SET_CURRENT:
//                ind = 0;
//                mc_interface_set_current(buffer_get_float32(rxmsg.data8, 1e3, &ind));
//                timeout_reset();
//                break;
//
//            case CAN_PACKET_SET_CURRENT_BRAKE:
//                ind = 0;
//                mc_interface_set_brake_current(buffer_get_float32(rxmsg.data8, 1e3, &ind));
//                timeout_reset();
//                break;
//
//            case CAN_PACKET_SET_RPM:
//                ind = 0;
//                mc_interface_set_pid_speed(buffer_get_float32(rxmsg.data8, 1e0, &ind));
//                timeout_reset();
//                break;
//
//            case CAN_PACKET_SET_POS:
//                ind = 0;
//                mc_interface_set_pid_pos(buffer_get_float32(rxmsg.data8, 1e6, &ind));
//                timeout_reset();
//                break;
//
//            case CAN_PACKET_FILL_RX_BUFFER:
//                memcpy(CAN_RxBuffer + rxmsg.data8[0], rxmsg.data8 + 1, rxmsg.DLC - 1);
//                break;
//
//            case CAN_PACKET_FILL_RX_BUFFER_LONG:
//                rxbuf_ind = (unsigned int)rxmsg.data8[0] << 8;
//                rxbuf_ind |= rxmsg.data8[1];
//                if (rxbuf_ind < RX_BUFFER_SIZE)
//                {
//                    memcpy(CAN_RxBuffer + rxbuf_ind, rxmsg.data8 + 2, rxmsg.DLC - 2);
//                }
//                break;
//
//            case CAN_PACKET_PROCESS_RX_BUFFER:
//                ind = 0;
//                rx_buffer_last_id = rxmsg.data8[ind++];
//                commands_send = rxmsg.data8[ind++];
//                rxbuf_len = (unsigned int)rxmsg.data8[ind++] << 8;
//                rxbuf_len |= (unsigned int)rxmsg.data8[ind++];
//                if (rxbuf_len > RX_BUFFER_SIZE)
//                {
//                    break;
//                }
//
//                crc_high = rxmsg.data8[ind++];
//                crc_low = rxmsg.data8[ind++];
//
//                if (crc16(CAN_RxBuffer, rxbuf_len) == ((unsigned short)crc_high << 8 | (unsigned short)crc_low))
//                {
//                    switch (commands_send)
//                    {
//                    case 0:
//                        commands_process_packet(CAN_RxBuffer, rxbuf_len, send_packet_wrapper);
//                        break;
//                    case 1:
//                        commands_send_packet(CAN_RxBuffer, rxbuf_len);
//                        break;
//                    case 2:
//                        commands_process_packet(CAN_RxBuffer, rxbuf_len, 0);
//                        break;
//                    default:
//                        break;
//                    }
//                }
//                break;
//
//            case CAN_PACKET_PROCESS_SHORT_BUFFER:
//                ind = 0;
//                rx_buffer_last_id = rxmsg.data8[ind++];
//                commands_send = rxmsg.data8[ind++];
//
//                switch (commands_send)
//                {
//                case 0:
//                    commands_process_packet(rxmsg.data8 + ind, rxmsg.DLC - ind, send_packet_wrapper);
//                    break;
//                case 1:
//                    commands_send_packet(rxmsg.data8 + ind, rxmsg.DLC - ind);
//                    break;
//                case 2:
//                    commands_process_packet(rxmsg.data8 + ind, rxmsg.DLC - ind, 0);
//                    break;
//                default:
//                    break;
//                }
//                break;
//
//            case CAN_PACKET_SET_CURRENT_REL:
//                ind = 0;
//                mc_interface_set_current_rel(buffer_get_float32(rxmsg.data8, 1e5, &ind));
//                timeout_reset();
//                break;
//
//            case CAN_PACKET_SET_CURRENT_BRAKE_REL:
//                ind = 0;
//                mc_interface_set_brake_current_rel(buffer_get_float32(rxmsg.data8, 1e5, &ind));
//                timeout_reset();
//                break;
//
//            case CAN_PACKET_SET_CURRENT_HANDBRAKE:
//                ind = 0;
//                mc_interface_set_handbrake(buffer_get_float32(rxmsg.data8, 1e3, &ind));
//                timeout_reset();
//                break;
//
//            case CAN_PACKET_SET_CURRENT_HANDBRAKE_REL:
//                ind = 0;
//                mc_interface_set_handbrake_rel(buffer_get_float32(rxmsg.data8, 1e5, &ind));
//                timeout_reset();
//                break;
//
//            case CAN_PACKET_PING:
//            {
//                uint8_t buffer[1];
//                buffer[0] = app_get_configuration()->controller_id;
//                comm_can_transmit_eid(rxmsg.data8[0] |
//                                          ((uint32_t)CAN_PACKET_PONG << 8),
//                                      buffer, 1);
//            }
//            break;
//
//            case CAN_PACKET_PONG:
//                // rxmsg.data8[0]; // Sender ID
//                if (ping_tp)
//                {
//                    chEvtSignal(ping_tp, 1 << 29);
//                }
//                break;
//
//            case CAN_PACKET_DETECT_APPLY_ALL_FOC:
//            {
//                ind = 1;
//                bool activate_status = rxmsg.data8[ind++];
//                float max_power_loss = buffer_get_float32(rxmsg.data8, 1e3, &ind);
//                int res = conf_general_detect_apply_all_foc(max_power_loss, true, false);
//                if (res >= 0 && activate_status)
//                {
//                    app_configuration appconf = *app_get_configuration();
//
//                    if (appconf.send_can_status != CAN_STATUS_1_2_3_4)
//                    {
//                        appconf.send_can_status = CAN_STATUS_1_2_3_4;
//                        conf_general_store_app_configuration(&appconf);
//                        app_set_configuration(&appconf);
//                    }
//                }
//
//                int8_t buffer[1];
//                buffer[0] = res;
//                comm_can_transmit_eid(rxmsg.data8[0] |
//                                          ((uint32_t)CAN_PACKET_DETECT_APPLY_ALL_FOC_RES << 8),
//                                      (uint8_t *)buffer, 1);
//            }
//            break;
//
//            case CAN_PACKET_DETECT_APPLY_ALL_FOC_RES:
//            {
//                detect_all_foc_res[detect_all_foc_res_index++] = (int8_t)rxmsg.data8[0];
//                detect_all_foc_res_index %= sizeof(detect_all_foc_res);
//            }
//            break;
//
//            case CAN_PACKET_CONF_CURRENT_LIMITS:
//            case CAN_PACKET_CONF_STORE_CURRENT_LIMITS:
//            {
//                ind = 0;
//                float min = buffer_get_float32(rxmsg.data8, 1e3, &ind);
//                float max = buffer_get_float32(rxmsg.data8, 1e3, &ind);
//                mc_configuration mcconf = *mc_interface_get_configuration();
//
//                if (mcconf.l_current_min != min || mcconf.l_current_max != max)
//                {
//                    mcconf.l_current_min = min;
//                    mcconf.l_current_max = max;
//
//                    if (cmd == CAN_PACKET_CONF_STORE_CURRENT_LIMITS)
//                    {
//                        conf_general_store_mc_configuration(&mcconf);
//                    }
//
//                    mc_interface_set_configuration(&mcconf);
//                }
//            }
//            break;
//
//            case CAN_PACKET_CONF_CURRENT_LIMITS_IN:
//            case CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN:
//            {
//                ind = 0;
//                float min = buffer_get_float32(rxmsg.data8, 1e3, &ind);
//                float max = buffer_get_float32(rxmsg.data8, 1e3, &ind);
//                mc_configuration mcconf = *mc_interface_get_configuration();
//
//                if (mcconf.l_in_current_min != min || mcconf.l_in_current_max != max)
//                {
//                    mcconf.l_in_current_min = min;
//                    mcconf.l_in_current_max = max;
//
//                    if (cmd == CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN)
//                    {
//                        conf_general_store_mc_configuration(&mcconf);
//                    }
//
//                    mc_interface_set_configuration(&mcconf);
//                }
//            }
//            break;
//
//            case CAN_PACKET_CONF_FOC_ERPMS:
//            case CAN_PACKET_CONF_STORE_FOC_ERPMS:
//            {
//                ind = 0;
//                float foc_openloop_rpm = buffer_get_float32(rxmsg.data8, 1e3, &ind);
//                float foc_sl_erpm = buffer_get_float32(rxmsg.data8, 1e3, &ind);
//                mc_configuration mcconf = *mc_interface_get_configuration();
//
//                if (mcconf.foc_openloop_rpm != foc_openloop_rpm ||
//                    mcconf.foc_sl_erpm != foc_sl_erpm)
//                {
//                    mcconf.foc_openloop_rpm = foc_openloop_rpm;
//                    mcconf.foc_sl_erpm = foc_sl_erpm;
//
//                    if (cmd == CAN_PACKET_CONF_STORE_FOC_ERPMS)
//                    {
//                        conf_general_store_mc_configuration(&mcconf);
//                    }
//
//                    mc_interface_set_configuration(&mcconf);
//                }
//            }
//            break;
//
//            default:
//                break;
//            }
//        }
//
//        switch (cmd)
//        {
//        case CAN_PACKET_STATUS:
//            for (int i = 0; i < CAN_STATUS_MSGS_TO_STORE; i++)
//            {
//                can_status_msg *stat_tmp = &stat_msgs[i];
//                if (stat_tmp->id == id || stat_tmp->id == -1)
//                {
//                    ind = 0;
//                    stat_tmp->id = id;
//                    stat_tmp->rx_time = chVTGetSystemTime();
//                    stat_tmp->rpm = (float)buffer_get_int32(rxmsg.data8, &ind);
//                    stat_tmp->current = (float)buffer_get_int16(rxmsg.data8, &ind) / 10.0;
//                    stat_tmp->duty = (float)buffer_get_int16(rxmsg.data8, &ind) / 1000.0;
//                    break;
//                }
//            }
//            break;
//
//        case CAN_PACKET_STATUS_2:
//            for (int i = 0; i < CAN_STATUS_MSGS_TO_STORE; i++)
//            {
//                can_status_msg_2 *stat_tmp_2 = &stat_msgs_2[i];
//                if (stat_tmp_2->id == id || stat_tmp_2->id == -1)
//                {
//                    ind = 0;
//                    stat_tmp_2->id = id;
//                    stat_tmp_2->rx_time = chVTGetSystemTime();
//                    stat_tmp_2->amp_hours = (float)buffer_get_int32(rxmsg.data8, &ind) / 1e4;
//                    stat_tmp_2->amp_hours_charged = (float)buffer_get_int32(rxmsg.data8, &ind) / 1e4;
//                    break;
//                }
//            }
//            break;
//
//        case CAN_PACKET_STATUS_3:
//            for (int i = 0; i < CAN_STATUS_MSGS_TO_STORE; i++)
//            {
//                can_status_msg_3 *stat_tmp_3 = &stat_msgs_3[i];
//                if (stat_tmp_3->id == id || stat_tmp_3->id == -1)
//                {
//                    ind = 0;
//                    stat_tmp_3->id = id;
//                    stat_tmp_3->rx_time = chVTGetSystemTime();
//                    stat_tmp_3->watt_hours = (float)buffer_get_int32(rxmsg.data8, &ind) / 1e4;
//                    stat_tmp_3->watt_hours_charged = (float)buffer_get_int32(rxmsg.data8, &ind) / 1e4;
//                    break;
//                }
//            }
//            break;
//
//        case CAN_PACKET_STATUS_4:
//            for (int i = 0; i < CAN_STATUS_MSGS_TO_STORE; i++)
//            {
//                can_status_msg_4 *stat_tmp_4 = &stat_msgs_4[i];
//                if (stat_tmp_4->id == id || stat_tmp_4->id == -1)
//                {
//                    ind = 0;
//                    stat_tmp_4->id = id;
//                    stat_tmp_4->rx_time = chVTGetSystemTime();
//                    stat_tmp_4->temp_fet = (float)buffer_get_int16(rxmsg.data8, &ind) / 10.0;
//                    stat_tmp_4->temp_motor = (float)buffer_get_int16(rxmsg.data8, &ind) / 10.0;
//                    stat_tmp_4->current_in = (float)buffer_get_int16(rxmsg.data8, &ind) / 10.0;
//                    stat_tmp_4->pid_pos_now = (float)buffer_get_int16(rxmsg.data8, &ind) / 50.0;
//                    break;
//                }
//            }
//            break;
//
//        case CAN_PACKET_STATUS_5:
//            for (int i = 0; i < CAN_STATUS_MSGS_TO_STORE; i++)
//            {
//                can_status_msg_5 *stat_tmp_5 = &stat_msgs_5[i];
//                if (stat_tmp_5->id == id || stat_tmp_5->id == -1)
//                {
//                    ind = 0;
//                    stat_tmp_5->id = id;
//                    stat_tmp_5->rx_time = chVTGetSystemTime();
//                    stat_tmp_5->tacho_value = buffer_get_int32(rxmsg.data8, &ind);
//                    stat_tmp_5->v_in = (float)buffer_get_int16(rxmsg.data8, &ind) / 1e1;
//                    break;
//                }
//            }
//            break;
//
//        default:
//            break;
//        }
//    }
//    else
//    {
//        reutrn;
//    }
//#endif
//}

#endif