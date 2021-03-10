#ifndef __SIMPLELIB_H
#define __SIMPLELIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "simplelib_cfg.h"

#ifdef SLIB_USE_CMD
#include "usart.h"
#include "cmd.h"
#include "cmd_func.h"
#endif

#ifdef SLIB_USE_CAN
#include "can_utils.h"
#include "can_func.h"
#endif

#ifdef SLIB_USE_NRF
#include "nrf24l01.h"
#endif

void SimpleLib_Init(UART_HandleTypeDef *cmd_usart, CAN_HandleTypeDef *hcan);
void SimpleLib_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* __SIMPLELIB_H */