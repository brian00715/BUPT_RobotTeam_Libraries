/*******************************************************************************
 * Copyright:		BUPT
 * File Name:		nrf24l01.h
 * Description:		NRF24L01模块驱动，参考本杰明电调
 * Author:			ZeroVoid
 * Version:			0.1
 * Data:			2019/09/29 Sun 22:06
 * Note:			HAL stm32F4 1.24.1 版本兼容
 *******************************************************************************/
#ifndef __NRF24L01_H
#define __NRF24L01_H

#ifdef __cplusplus
extern "C" {
#endif
// TODO: ZeroVoid	due:10/07	finish glossary
/**
 * @brief	Glossary Note
 * ACK      Acknowledgement
 * ART      Auto Re-Transmit
 * CRC      Cyclic Redundancy Check
 */

/*******************************************************************************
 * Include
 *******************************************************************************/
#include "simplelib_cfg.h"

#ifdef SLIB_USE_NRF
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>


/*******************************************************************************
 * NRF Macros Define
 *******************************************************************************/
/**
 * @brief	NRF IRQ Enable
 */
#define NRF_IT_ENABLE                       0           
#define NRF_IT_DISABLE                      1

/**
 * @brief	RX FIFO empty
 * @note	Status Resigster PX_P_NO 使用
 */
#define NRF_RX_FIFO_EMPTY					(0x07)


/*******************************************************************************
 * NRF Data Type
 *******************************************************************************/
/**
 * @brief	NRF CRC encoding scheme value
 * @note	REG_CONFIG CRCO 0-1bit 1-2bit
 */
typedef enum {
	NRF_CRC_1B = 0,
	NRF_CRC_2B,
	NRF_CRC_DISABLED
} NRF_CRC;

/**
 * @brief	NRF Power
 * @note	not NRF RF Power
 */
typedef enum {
	NRF_POWER_DOWN = 0,
	NRF_POWER_UP = 2
} NRF_POWER;

/**
 * @brief	NRF Mode
 */
typedef enum {
    NRF_PTX = 0,
    NRF_PRX
} NRF_MODE;

/**
 * @brief	NRF Address width
 * @note	3-5 Bytes
 */
typedef enum {
    NRF_AW_3 = 0x01,
    NRF_AW_4,
    NRF_AW_5
} NRF_AW;

/**
 * @brief	Auto Retransmit Delay
 */
typedef enum {
	NRF_RETR_DELAY_250US = 0,
	NRF_RETR_DELAY_500US,
	NRF_RETR_DELAY_750US,
	NRF_RETR_DELAY_1000US,
	NRF_RETR_DELAY_1250US,
	NRF_RETR_DELAY_1500US,
	NRF_RETR_DELAY_1750US,
	NRF_RETR_DELAY_2000US,
	NRF_RETR_DELAY_2250US,
	NRF_RETR_DELAY_2500US,
	NRF_RETR_DELAY_2750US,
	NRF_RETR_DELAY_3000US,
	NRF_RETR_DELAY_3250US,
	NRF_RETR_DELAY_3500US,
	NRF_RETR_DELAY_3750US,
	NRF_RETR_DELAY_4000US
} NRF_RETR_DELAY;

/**
 * @brief	NRF RF Data Rate speed
 */
typedef enum {
	NRF_SPEED_250K = 0,
	NRF_SPEED_1M,
	NRF_SPEED_2M
} NRF_SPEED;

/**
 * @brief	RF output power in TX mode
 */
typedef enum {
	NRF_RF_POWER_M18DBM = 0,
	NRF_RF_POWER_M12DBM,
	NRF_RF_POWER_M6DBM,
	NRF_RF_POWER_0DBM,
} NRF_RF_POWER;

/**
 * @brief	NRF pipe number
 */
typedef enum {
	NRF_PIPE_0 = 0,
	NRF_PIPE_1,
	NRF_PIPE_2,
	NRF_PIPE_3,
	NRF_PIPE_4,
	NRF_PIPE_5
} NRF_PIPE;

/**
 * @brief	NRF main function control state
 * @note	目前只完成了回调部分函数
 */
typedef enum nrf_flow_control {
	NRF_IDLE,
    NRF_TX_CALLBACK,
    NRF_RX_CALLBACK,
    NRF_MAX_RT_CALLBACK,
    NRF_COMM_SEND
} NRF_FLOW_STATE;

/**
 * @brief	NRF config
 * @note	NRF 初始化使用
 */
typedef struct {
	NRF_SPEED speed;
	NRF_POWER power;
	NRF_RF_POWER rf_power;
	NRF_CRC crc_type;
	NRF_RETR_DELAY retry_delay;
	uint8_t retries;
	uint8_t channel;
	uint8_t pipes;
	uint8_t address[5];
	NRF_AW addr_len;
	bool send_crc_ack;
} NRF_ConfigTypeDef;

/**
 * @brief	NRF handle
 */
typedef struct {
	SPI_HandleTypeDef *hspi;
	uint8_t *tx_addr;
	uint8_t *tx_data;
	uint8_t tx_len;

	uint8_t **rx_addr;
	uint8_t *rx_data;
	uint8_t rx_len;
	NRF_PIPE rx_pipe;

	uint8_t nrf_data_from;
	uint8_t nrf_data_to;
	uint8_t nrf_addr_len;
} NRF_Handle;

/*******************************************************************************
 * NRF Val
 *******************************************************************************/
extern uint8_t nrf_rx_data[32];
extern uint8_t nrf_tx_data[32];
extern uint8_t nrf_tx_addr[5];
extern uint8_t nrf_rx_addr[6][5];
extern NRF_FLOW_STATE nrf_flow_state;
extern uint8_t nrf_all_can_send;
extern NRF_Handle nrf_handle;
// extern NRF_AW nrf_addr_width;
extern bool nrf_rx_addr_set[6];


/*******************************************************************************
 * NRF Register Map
 *******************************************************************************/
#define NRF_REG_CONFIG						0x00 // Configuration Register
#define NRF_REG_EN_AA						0x01 // Auto Acknowledgment
#define NRF_REG_EN_RXADDR					0x02 // Enabled RX Addresses
#define NRF_REG_SETUP_AW					0x03 // Setup of Address Widths
#define	NRF_REG_SETUP_RETR					0x04 // Setup of Automatic Retransmission
#define NRF_REG_RF_CH						0x05 // RF Channel
#define NRF_REG_RF_SETUP					0x06 // RF Setup Register
#define NRF_REG_STATUS						0x07 // Status Register
#define NRF_REG_OBSERVE_TX					0x08 // Transmit observe register
#define NRF_REG_RPD							0x09 // Receive power detector
#define NRF_REG_RX_ADDR_P0					0x0A // Receive address data pipe 0
#define NRF_REG_RX_ADDR_P1					0x0B // Receive address data pipe 1
#define NRF_REG_RX_ADDR_P2					0x0C // Receive address data pipe 2
#define NRF_REG_RX_ADDR_P3					0x0D // Receive address data pipe 3
#define NRF_REG_RX_ADDR_P4					0x0E // Receive address data pipe 4
#define NRF_REG_RX_ADDR_P5					0x0F // Receive address data pipe 5
#define NRF_REG_TX_ADDR						0x10 // Transmit address
#define NRF_REG_RX_PW_P0					0x11 // Number of bytes in RX payload
#define NRF_REG_RX_PW_P1					0x12 // Number of bytes in RX payload
#define NRF_REG_RX_PW_P2					0x13 // Number of bytes in RX payload
#define NRF_REG_RX_PW_P3					0x14 // Number of bytes in RX payload
#define NRF_REG_RX_PW_P4					0x15 // Number of bytes in RX payload
#define NRF_REG_RX_PW_P5					0x16 // Number of bytes in RX payload
#define NRF_REG_FIFO_STATUS					0x17 // FIFO Status Register
#define NRF_REG_DYNPD						0x1C // Enable dynamic payload length
#define NRF_REG_FEATURE						0x1D // Feature Register

/*******************************************************************************
 * NRF SPI Commands
 *******************************************************************************/
#define NRF_CMD_READ_REGISTER				0x00 // 000AAAAA: AAAAA 5 bit register address
#define NRF_CMD_WRITE_REGISTER				0x20 // 001AAAAA: AAAAA 5 bit register address
#define NRF_CMD_READ_RX_PAYLOAD				0x61 // Read RX-payload
#define NRF_CMD_WRITE_TX_PAYLOAD			0xA0 // Write TX-payload
#define NRF_CMD_FLUSH_TX					0xE1 // Flush TX FIFO
#define NRF_CMD_FLUSH_RX					0xE2 // Flush RX FIFO
#define NRF_CMD_REUSE_TX_PL					0xE3 // Reuse last transmitted payload
//#define NRF_CMD_ACTIVATE					0b01010000 // Activate features 手册上没看到
#define NRF_CMD_READ_RX_PAYLOAD_WIDTH		0x60 // Read RX-payload width
#define NRF_CMD_WRITE_ACK_PAYLOAD			0xA8 // 10101PPP:Write payload for ACK packet,PPP pipe 000 - 101
#define NRF_CMD_WRITE_TX_PAYLOAD_NO_ACK		0xB0 // Disables AUTOACK on packet
#define NRF_CMD_NOP							0xFF // NOP

/*******************************************************************************
 * NRF Register Masks
 *******************************************************************************/
/* Pipe Number Masks -----------------------------------------------------*/
#define NRF_MASK_PIPE5						(1<<5)
#define NRF_MASK_PIPE4						(1<<4)
#define NRF_MASK_PIPE3						(1<<3)
#define NRF_MASK_PIPE2						(1<<2)
#define NRF_MASK_PIPE1						(1<<1)
#define NRF_MASK_PIPE0						(1<<0)

/* Config Register Masks -----------------------------------------------------*/
#define NRF_CONFIG_MASK_RX_DR				(1<<6) // Mask interrupt cause by RX_DR. 0 Valid
#define NRF_CONFIG_MASK_TX_DS				(1<<5) // Mask interrupt cause by TX_DS. 0 Valid
#define NRF_CONFIG_MASK_MAX_RT				(1<<4) // Mask interrupt cause by MAX_RT. 0 Valid
#define NRF_CONFIG_EN_CRC					(1<<3) // Enable CRC. Force high if one of the bits in EN_AA is high
#define NRF_CONFIG_CRCO						(1<<2) // CRC encoding scheme
#define NRF_CONFIG_PWR_UP					(1<<1) // 1:Power up, 0:Power down
#define NRF_CONFIG_PRIM_RX					(1<<0) // 1:PRX, 0:PTX

/* RETR Setup Masks -----------------------------------------------------*/
#define NRF_SETUP_RETR_ARD					(1<<4 | 1<<5 | 1<<6 | 1<<7) // Auto Retransimit Delay
#define NRF_SETUP_RETR_ARC					(1<<0 | 1<<1 | 1<<2 | 1<<3) // Auto Retransimit Count

/* RF Setup Masks -----------------------------------------------------*/
#define NRF_RF_SETUP_CONT_WAVE              (1<<7) // Enables continuous carrier transmit when high
#define NRF_RF_SETUP_RF_DR_LOW				(1<<5) // Set RF Data Rate to 260kbps
#define NRF_RF_SETUP_PLL_LOCK               (1<<4) // Only usee in test
#define NRF_RF_SETUP_RF_DR_HIGH				(1<<3) // Select between the high speed data rates
#define NRF_RF_SETUP_RF_PWR					(1<<1 | 1<<2) // Set RF output power in TX mode

/* Status Register Masks -----------------------------------------------------*/
#define NRF_STATUS_RX_DR					(1<<6) // Data Ready RX FIFO interrupt
#define NRF_STATUS_TX_DS					(1<<5) // Data Sent TX FIFO interrupt. If AUTO_ACK is activated, set high only when ACK is received.
#define NRF_STATUS_MAX_RT					(1<<4) // Maximum number of TX retransmits interrupt
#define NRF_STATUS_RX_P_NO					(1<<1 | 1<<2 | 1<<3) // Data pipe number for the payload available for reading from RX_FIFO
#define NRF_STATUS_TX_FULL					(1<<0) // TX FIFO full flag. 1:full
#define NRF_STATUS_IRQ 						(NRF_STATUS_MAX_RT | NRF_STATUS_TX_DS | NRF_STATUS_RX_DR)

/* FIFO status Register Masks -----------------------------------------------------*/
#define NRF_FIFO_TX_REUSE					(1<<6) // Used for a PTX device.
#define NRF_FIFO_TX_FULL					(1<<5) // 1:full
#define NRF_FIFO_TX_EMPTY					(1<<4) // 1:empty
#define NRF_FIFO_RX_FULL					(1<<1) // 1:full
#define NRF_FIFO_RX_EMPTY					(1<<0) // 1:empty

/* Observe Register Masks -----------------------------------------------------*/
#define NRF_PLOS_CNT					 	(0xF0) // Count lost packets
#define NRF_PLOS_CNT_POS					(4)   
#define NRF_ARC_CNT							(0x0F) // Count retransmitted packets

/* Feature Register Masks -----------------------------------------------------*/
#define NRF_FEATURE_DPL						(1<<2) // Enable Dynamic Payload Length
#define NRF_FEATURE_ACK_PAYLOAD				(1<<1) // Enable Payload with ACK
#define NRF_FEATURE_DYN_ACK					(1<<0) // Enable the W_TX_PAYLOAD_NOACK command


/*******************************************************************************
 * NRF Macro Functions
 *******************************************************************************/
#define NRF_STATUS_GET_TX_FULL(s)			(s & NRF_STATUS_TX_FULL)
#define NRF_STATUS_GET_RX_P_NO(s)			((s & NRF_STATUS_RX_P_NO) >> 1)
#define NRF_STATUS_GET_MAX_RT(s)			((s & NRF_STATUS_MAX_RT) >> 4)
#define NRF_STATUS_GET_TX_DS(s)				((s & NRF_STATUS_TX_DS) >> 5)
#define NRF_STATUS_GET_RX_DR(s)				((s & NRF_STATUS_RX_DR) >> 6)

/*******************************************************************************
 * NRF Mid Functions
 *******************************************************************************/
void nrf_init(NRF_ConfigTypeDef *config);
void nrf_stop(void);
uint8_t nrf_send_data(uint8_t *data, int len);
uint8_t nrf_read_rx_data(uint8_t *data, uint8_t *len, NRF_PIPE *pipe); 
void nrf_set_tx_addr(uint8_t *addr, uint8_t addr_len);
void nrf_set_rx_addr(NRF_PIPE pipe, uint8_t *addr, uint8_t len);
void nrf_set_addr_width(uint8_t width);
void nrf_get_tx_addr(uint8_t** addr, uint8_t *len);
void nrf_get_rx_addr(NRF_PIPE pipe, uint8_t** addr, uint8_t *len);
void nrf_irq_handle(void);
void _nrf_receive_callback(uint8_t *data, int len);
void _nrf_send_callback(void);
void _nrf_max_rt_callback(void);

/*******************************************************************************
 * NRF Driver Functions
 *******************************************************************************/
/* 0x00 Configuration Register -----------------------------------------------------*/
void _nrf_set_crc_type(NRF_CRC crc_type);
void _nrf_set_power(NRF_POWER power);
void _nrf_set_mode(NRF_MODE mode);

/* 0x01-x03 -----------------------------------------------------*/
void _nrf_enable_pipe_autoack(uint8_t pipes);
void _nrf_enable_pipe_address(uint8_t pipes);
void _nrf_disable_pipe_address(uint8_t pipes);
void _nrf_set_address_width(NRF_AW aw);
NRF_AW _nrf_get_address_width(void);

/* 0x04 -----------------------------------------------------*/
void _nrf_set_retr_delay(NRF_RETR_DELAY delay);
void _nrf_set_retr_retries(uint8_t retries);

/* 0x05 -----------------------------------------------------*/
void _nrf_set_frequency(int freq);
int _nrf_get_frequency(void);

/* 0x06 -----------------------------------------------------*/
void _nrf_set_speed(NRF_SPEED speed);
void _nrf_set_rf_power(NRF_RF_POWER power);

/* 0x07 -----------------------------------------------------*/
uint8_t _nrf_get_status(void);
void _nrf_clear_irq(void);
void _nrf_clear_rx_irq(void);
void _nrf_clear_tx_irq(void);
void _nrf_clear_maxrt_irq(void);

/* 0x08 Transmit Observe Register -----------------------------------------------------*/
uint8_t _nrf_get_plos_cnt(void);
uint8_t _nrf_get_arc_cnt(void);

/* 0x09 -----------------------------------------------------*/
uint8_t _nrf_rx_power_detect(void);

/* 0x0A-0x0F -----------------------------------------------------*/
void _nrf_set_rx_addr(NRF_PIPE pipe, uint8_t *address, uint8_t len);

/* 0x10 -----------------------------------------------------*/
void _nrf_set_tx_addr(uint8_t *address, uint8_t addr_len);

/* 0x11-0x16 -----------------------------------------------------*/
uint8_t _nrf_get_payload_width(void);
uint8_t _nrf_get_payload_width_pipe(NRF_PIPE pipe);

/* 0x17 -----------------------------------------------------*/
uint8_t _nrf_get_fifo_status(void);

void _nrf_write_tx_payload(uint8_t *data, uint8_t length);
void _nrf_write_tx_payload_no_ack(uint8_t *data, uint8_t length);
void _nrf_write_ack_payload(NRF_PIPE pipe, uint8_t *data, uint8_t length);
void _nrf_read_rx_payload(uint8_t *data, uint8_t length);

/* 0x1C-0x1D -----------------------------------------------------*/
void _nrf_enable_pipe_dlp(uint8_t pipes);
void _nrf_enable_features(uint8_t features);

/* Commands -----------------------------------------------------*/
void _nrf_flush_tx(void);
void _nrf_flush_rx(void);
void _nrf_flush_all(void);

#endif // SLIB_USE_NRF


#ifdef __cplusplus
}
#endif

#endif /* __NRF24L01_H */
