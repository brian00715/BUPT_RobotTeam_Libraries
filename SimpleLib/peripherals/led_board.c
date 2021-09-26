/*******************************************************************************
Copyright:      BUPT
File name:      led.c
Description:    控制sk6812灯板的程序
Author:         ANZ
Version：       1.0
Data:           2020、10、18
*******************************************************************************/

#include "led_board.h"
#ifdef USE_LED_BOARD

/**
 * @brief 给led灯板发送can消息
 * @param <cmd1,cmd2,cmd3,cmd4,cmd5,cmd6,cmd7,cmd8>
 **/
void LedBoard_send_message(int16_t cmd1, int16_t cmd2, int16_t cmd3, int16_t cmd4, int16_t cmd5, int16_t cmd6, int16_t cmd7, int16_t cmd8)
{
    uint8_t Data[8];
    Data[0] = cmd1;
    Data[1] = cmd2;
    Data[2] = cmd3;
    Data[3] = cmd4;
    Data[4] = cmd5;
    Data[5] = cmd6;
    Data[6] = cmd7;
    Data[7] = cmd8;
    CAN_Message_u led_data;
    for (int i = 0; i <= 7; i++)
    {
        led_data.ui8[i] = Data[i];
    }

    CAN_SendMsg(10, &led_data);
}

static uint8_t cmd[8] = {0};
void led_control(int state)
{
    if (state <= 100)
    {
        cmd[6] = state % 10;        // 个位
        cmd[7] = (int)(state / 10); // 十位
    }
    LedBoard_send_message(cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6], cmd[7]);
}

void LED_Init()
{
    for (int i = 0; i < 8; i++)
    {
        cmd[i] = 0;
    }
}
#endif