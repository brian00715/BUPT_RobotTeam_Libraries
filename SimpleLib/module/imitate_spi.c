/*******************************************************************************
 * Copyright:		BUPT
 * File Name:		imitate_spi.c
 * Author:			2018
 * Description:		None
 * Bug:				None
 * Version:			0.1
 * Data:			2019/10/18 Fri 22:02
 * Todo:			软件spi底层函数
 *******************************************************************************/
#include "imitate_spi.h"
#ifdef SL_IMI_SPI

#include "gpio.h"
#include "tim.h"
#include "usart.h"

void Delay(uint32_t delay_time)
{
  TIM6->CNT=0;
  HAL_TIM_Base_Start(&htim6);
  while(TIM6->CNT < delay_time);
  HAL_TIM_Base_Stop(&htim6);
}

//!函数表未改
/**
 * @brief	软件spi1读写函数 CPOL=0 CPHA=0
 * @param	uint8_t writeBuffer 发送的数据
 * @return	收到的数据
 */
uint8_t ispi1_write_read_byte(uint8_t writeBuffer) {
    int i;
    uint8_t Buffer = 0;
    for (i = 7; i >= 0; i--) {
    HAL_GPIO_WritePin(
        iSPI1_MOSI_GPIO_Port, iSPI1_MOSI_Pin,
        (GPIO_PinState)(writeBuffer & (1 << i)));  //从高位7到低位0进行串行写入
    // Delay(5);
    iSPI1_CLK_Enable();
    Delay(5);
    iSPI1_CLK_Disable();  //下降沿输样
    GPIO_PinState temp = HAL_GPIO_ReadPin(iSPI1_MISO_GPIO_Port, iSPI1_MISO_Pin);
    Buffer = (Buffer << 1) | temp;  //从高位7到低位0进行串行读出
    }
    return Buffer;
}

/**
 * @brief	软件spi1读写函数 CPOL=0 CPHA=0
 * @param	uint8_t writeBuffer 发送的数据
 * @return	收到的数据
 */
void ispi1_write_read(uint8_t* writeBuffer, uint8_t* readBuffer, int len) {
  int i;
  iSPI1_CS_Enable();  //从设备使能有效，通信开始
  Delay(10);
  for (i = 0; i < len; i++)
    readBuffer[i] = ispi1_write_read_byte(writeBuffer[i]);
  Delay(10);
  iSPI1_CS_Disable();
  Delay(10);
}

/**
 * @brief	软件spi1读写函数 CPOL=0 CPHA=0
 * @param	uint8_t writeBuffer 发送的数据
 * @return	收到的数据
 */
uint8_t ispi3_write_read_byte(uint8_t writeBuffer) {
  int i;
  uint8_t Buffer = 0;
  for (i = 7; i >= 0; i--) {
    HAL_GPIO_WritePin(
        iSPI3_MOSI_GPIO_Port, iSPI3_MOSI_Pin,
        (GPIO_PinState)(writeBuffer & (1 << i)));  //从高位7到低位0进行串行写入
    // Delay(5);
    iSPI3_CLK_Enable();
    Delay(5);
    iSPI3_CLK_Disable();  //下降沿输样
    GPIO_PinState temp = HAL_GPIO_ReadPin(iSPI3_MISO_GPIO_Port, iSPI3_MISO_Pin);
    Buffer = (Buffer << 1) | temp;  //从高位7到低位0进行串行读出
    Delay(5);
  }
  return Buffer;
}

/**
 * @brief	软件spi1读写函数 CPOL=0 CPHA=0
 * @param	uint8_t writeBuffer 发送的数据
 * @return	收到的数据
 */
void ispi3_write_read(uint8_t* writeBuffer, uint8_t* readBuffer, int len) {
  int i;
  iSPI3_CS_Enable();  //从设备使能有效，通信开始
  Delay(10);
  for (i = 0; i < len; i++)
    readBuffer[i] = ispi3_write_read_byte(writeBuffer[i]);
  Delay(10);
  iSPI3_CS_Disable();
  Delay(10);
}

#endif // SL_IMI_SPI