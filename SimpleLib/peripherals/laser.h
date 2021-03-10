#ifndef __LASER_H
#define __LASER_H
#ifdef USE_LASER
#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"
#include "main.h"
#include "karman.h"
#include "adc.h"
#include "utils.h"
#include "handle.h"

  /*Define Area*/

#define AVERAGE_AMOUNT 10

  /*Struct Area*/

  typedef struct
  {
    uint32_t adc_l;
    uint32_t adc_r;
    uint32_t adc_s;
  } LASER_ADC;
  typedef struct
  {
    uint32_t ADC_value[AVERAGE_AMOUNT];
    int ADC_final;
    float distance;
    float FAR_distance;
    float NEAR_distance;
    float FAR_voltage;
    float NEAR_voltage;
    float k_param;
    float b_param;
  } LASER;

  /*Variable Area*/

  extern LASER laser_left;
  extern LASER laser_side;
  extern LASER laser_right;

  extern LASER_ADC laser_adc[AVERAGE_AMOUNT];

  extern PID_s laser_ypos_pid;
  extern PID_s laser_xpos_pid;

  extern int Laser_PrintADCValue_Flag;
  extern int Laser_PrintPos_Flag;

      /*Function Area*/

      void
      laser_calculate_kb(LASER *sensor);
  void laser_adc_split(LASER *laser_l, LASER *laser_r, LASER *laser_s);

  float laser_calculate_distance(LASER *sensor, Kal_Struct *kal_laser_distance, Kal_Struct *kal_laser_adc);
  float laser_calculate_x();
  float laser_calculate_y();
  float laser_calculate_angle();

  void laser_init();
  void laser_exe();
  void laser_print_distance();
  void Laser_PrintPos();
  void Laser_PrintADCValue();

#ifdef __cplusplus
}
#endif
#endif // USE_LASER
#endif /*__LASER_H */