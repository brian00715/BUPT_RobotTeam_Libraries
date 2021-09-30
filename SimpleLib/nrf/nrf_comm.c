/*******************************************************************************
 * Copyright:		BUPT
 * File Name:		nrf_comm.c
 * Description:		NRF
 * Author:			ZeroVoid
 * Version:			0.1
 * Data:			2019/11/08 Fri 21:06
 *******************************************************************************/
#include "nrf_comm.h"
#ifdef SLIB_USE_NRF_COMM

#ifdef SLIB_USE_CMD
#include "cmd.h"
#endif // SLIB_USE_CMD

#include <string.h>

void _nrf_receive_callback(uint8_t *data, int len);
void _nrf_send_callback(void);
void _nrf_max_rt_callback(void);
void nrf_comm_cmd(NRF_Handle *handle);

/**
 * @brief	NRF 基于协议发送函数
 * @param   data        发送数据
 * @param   len         数据长度
 * @param   data_deal   NRF_UART / NRF_CAN / NRF_SPI 组合
 *          e.g. NRF_UART | NRF_CAN
 */
void nrf_comm_send(uint8_t *data, int len, uint8_t data_deal) {
    nrf_handle.nrf_data_from = NRF_SPI;
    nrf_handle.nrf_data_to = data_deal;
    _nrf_comm_send(data, len);
}

void nrf_main(void) {
    switch(nrf_flow_state) {
    case NRF_COMM_SEND:
        _nrf_comm_send(nrf_handle.tx_data, nrf_handle.tx_len);
        nrf_flow_state = NRF_IDLE;
        break;

    case NRF_RX_CALLBACK:
        _nrf_receive_callback(nrf_handle.rx_data, nrf_handle.rx_len);
        nrf_flow_state = NRF_IDLE;
        break;
    case NRF_TX_CALLBACK:
        _nrf_send_callback();
        nrf_flow_state = NRF_IDLE;
        break;
    case NRF_MAX_RT_CALLBACK:
        _nrf_max_rt_callback();
        nrf_flow_state = NRF_IDLE;
        break;
    
    default:
        break;
    }
}


void _nrf_comm_send(uint8_t *data, int len) {
    data[0] = (nrf_handle.nrf_data_from << 4) | nrf_handle.nrf_data_to;
    nrf_handle.tx_data = data;
    nrf_handle.tx_len = len;
    nrf_send_data(data, len);
}

void _nrf_receive_callback(uint8_t *data, int len) {
    uint8_t tmp = (data[0] & 0x0F);
    if (tmp & NRF_UART) {
        #ifdef SLIB_USE_CMD
        uprintf_to(&huart1, (char*)(nrf_rx_data + NRF_PCK_HEADER_SIZE));
        #endif // SLIB_USE_CMD
        nrf_uart_receive_callback(data + NRF_PCK_HEADER_SIZE, len - NRF_PCK_HEADER_SIZE);
    }
    if (tmp & NRF_CAN) {
        nrf_can_receive_callback(data + NRF_PCK_HEADER_SIZE, len - NRF_PCK_HEADER_SIZE);
    }
    if (tmp & NRF_SPI) {
        nrf_comm_cmd(&nrf_handle);
        nrf_spi_receive_callback(data + NRF_PCK_HEADER_SIZE, len - NRF_PCK_HEADER_SIZE);
    }
    #ifdef SLIB_USE_NRF_DEBUG
    HAL_GPIO_TogglePin(IND_LED_GPIO_Port, IND_LED_Pin);
    #endif // SLIB_USE_NRF_DEBUG
    nrf_receive_callback(nrf_handle.rx_data, nrf_handle.rx_len);
}
void nrf_comm_cmd(NRF_Handle *handle) {
    uint8_t arg = handle->rx_data[NRF_PCK_HEADER_SIZE]&0x0F;
    uint8_t cmd = handle->rx_data[NRF_PCK_HEADER_SIZE] >> 4;
    switch (cmd) {
    case NRF_COMM_CMD_ALL_CAN:
        nrf_all_can_send = arg;
        break;
    
    default:
        break;
    }
}

void _nrf_send_callback(void) {
    #ifdef SLIB_USE_NRF_DEBUG
    HAL_GPIO_TogglePin(IND_LED_GPIO_Port, IND_LED_Pin);
    #endif // SLIB_USE_NRF_DEBUG
    nrf_send_callback(nrf_handle.tx_data, nrf_handle.tx_len);
}

void _nrf_max_rt_callback(void) {
    #ifdef SL_NRF_DEBUG_
    HAL_GPIO_TogglePin(IND_LED_GPIO_Port, IND_LED_Pin);
    #endif // SLIB_USE_NRF_DEBUG
    nrf_max_rt_callback(nrf_handle.tx_data, nrf_handle.tx_len);
}

__weak void nrf_spi_receive_callback(uint8_t *data, int len) {}
__weak void nrf_can_receive_callback(uint8_t *data, int len) {}
__weak void nrf_uart_receive_callback(uint8_t *data, int len) {}
__weak void nrf_receive_callback(uint8_t *data, int len) {}
__weak void nrf_send_callback(uint8_t *data, int len) {}
__weak void nrf_max_rt_callback(uint8_t *data, int len) {}

#ifdef SL_NRF_HW_CAN

void _can_rx_nrf_callback(uint32_t *id, CAN_Message_u *data) {
    nrf_handle.nrf_data_from = NRF_CAN;
    nrf_handle.nrf_data_to = NRF_UART | NRF_SPI;
    nrf_handle.tx_len = 9 + NRF_PCK_HEADER_SIZE;
    strncpy((char*)(nrf_handle.tx_data + NRF_PCK_HEADER_SIZE), (char*)id, 4);
    strncpy((char*)(nrf_handle.tx_data + NRF_PCK_HEADER_SIZE + 4), (char*)data, 8);
    _nrf_comm_send(nrf_handle.tx_data, nrf_handle.tx_len);
}

#endif // SL_NRF_HW_CAN

#endif // SLIB_USE_NRF_COMM