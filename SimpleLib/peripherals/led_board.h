
#ifndef __LED_H
#define __LED_H
#ifdef USE_LED_BOARD
#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "can_func.h"

    void LedBoard_send_message(int16_t cmd1, int16_t cmd2, int16_t cmd3, int16_t cmd4, int16_t cmd5, int16_t cmd6, int16_t cmd7, int16_t cmd8);
    void led_control(int state);
    void LED_Init();

#ifdef __cplusplus
}
#endif
#endif // USE_LED_BOARD
#endif /* __LED_H */