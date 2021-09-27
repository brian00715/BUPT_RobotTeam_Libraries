/*******************************************************************************
 * Copyright:		BUPT
 * File Name:		nrf_comm.h
 * Description:		NRF Communication Functions
 * Author:			ZeroVoid
 * Version:			0.1
 * Data:			2019/11/08 Fri 21:04
 * Encoding:		UTF-8
 *******************************************************************************/
#ifndef __NRF_COMM_H
#define __NRF_COMM_H

#ifdef __cplusplus
extern "C" {
#endif
#include "simplelib_cfg.h"
#ifdef SLIB_USE_NRF_COMM
#include "nrf24l01.h"
#include "can_utils.h"

/*******************************************************************************
 * NRF Communication Configuration
 *******************************************************************************/
/* NRF Hardware 选择-----------------------------------------------------*/
#define SL_NRF_HW_CAN                               0x10 // 启用CAN相关

/*******************************************************************************
 * NRF Flash Saved Data Offset
 *******************************************************************************/
#define NRF_TX_ADDR_OFFSET					        0 // 
#define NRF_RX_PIPE0_ADDR_OFFSET					5 // 
#define NRF_RX_PIPE1_ADDR_OFFSET					10 // 
#define NRF_RX_SET_OFFSET					        15 // 
#define NRF_RX_PIPE2_5_ADDR_OFFSET					21 // 
#define NRF_ADDR_LEN_OFFSET                         25
#define NRF_NAME_OFFSET					            26 //

/*******************************************************************************
 * NRF Protocol
 *******************************************************************************/
#define NRF_PCK_HEADER_SIZE					        2   // 数据包报头长度/Byte
#define NRF_CAN_SID                                 230 // ASCII NRF 加和
#define NRF_COMM_CMD_ALL_CAN                        10
typedef enum nrf_comm_way {
    NRF_UART = 0x1,
    NRF_CAN = 0x2,
    NRF_SPI = 0x4
} NRF_COMM_WAY;

/*******************************************************************************
 * NRF Protocol Val
 *******************************************************************************/

void nrf_main(void);
void nrf_comm_send(uint8_t *data, int len, uint8_t data_deal);
void _can_rx_nrf_callback(uint32_t *id, CAN_Message_u *data);
void _nrf_comm_send(uint8_t *data, int len);

/* Weak Functions -----------------------------------------------------*/
void nrf_spi_receive_callback(uint8_t *data, int len);
void nrf_can_receive_callback(uint8_t *data, int len);
void nrf_uart_receive_callback(uint8_t *data, int len);
void nrf_receive_callback(uint8_t *data, int len);
void nrf_send_callback(uint8_t *data, int len);
void nrf_max_rt_callback(uint8_t *data, int len);

#endif // SLIB_USE_NRF_COMM
#ifdef __cplusplus
}
#endif

#endif /* __NRF_COMM_H */