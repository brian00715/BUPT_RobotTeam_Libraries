/*******************************************************************************
 * Copyright:		BUPT
 * File Name:		flags.h
 * Description:		Flags 文件
 * Author:			ZeroVoid
 * Version:			0.1
 * Data:			2019/10/01 Tue 08:51
 *******************************************************************************/
#ifndef __FLAGS_H
#define __FLAGS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

extern int can_exc_callback_flag;
extern int DMA_RxOK_Flag;
extern uint8_t _5ms;
extern int send_wave_flag;
extern uint8_t test_flag;
extern uint8_t kick_test_flag;

#ifdef __cplusplus
}
#endif

#endif /* __FLAGS_H */
