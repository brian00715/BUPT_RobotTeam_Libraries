/**
  ******************************************************************************
  * @file           toolBoxScope.h
  * @author         ZohyCao
  * @version        1.0
  * @brief          ToolBox串口交互协议，波形显示独立版本，适用于ToolBox 1.04
  *                
  * 使用步骤：
  * 		适用toolBox_scope()函数发送波形数据即可，调用频率须低于200Hz
  *         数据包格式：第一位-------2（表示数据长度小于256）
  *                    第二位-------指令长度
  *                    第三位-------指令ID(103)
  *                    指令位
  *                    末三位-------crc低校验位
  *                    末二位-------crc高校验位
  *                    末一位-------3（结束标值）
  * 注意事项：
  *         数据精度1%
  *         为保持与VESC的兼容性，指令ID从100开始，详见COMM_PACK_ID
  ******************************************************************************
  */
#ifndef TOOLBOXSCOPE_H_
#define TOOLBOXSCOPE_H_

#include "simplelib_cfg.h"
#ifdef SLIB_USE_TOOLBOXSCOPE

/* Includes ---------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "usart.h"


/* Public Macro---------------------------------------------------*/
#define TOOLBOXSCOPE_UART (huart2)

/* Public Types---------------------------------------------------*/

/* Datatypes---------------------------------------------------*/

/* Crc Functions ---------------------------------------------------*/
unsigned short crc16(unsigned char *buf, unsigned int len);
/* Packet Functions ---------------------------------------------------*/
void toolBox_scope(float *dataArray,int dataNum);

#endif // SLIB_USE_TOOLBOXSCOPE

#endif /* TOOLBOXSCOPE_H_ */