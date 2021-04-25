#ifndef __AS5047P_H
#define __AS5047P_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "simplelib_cfg.h"
#ifdef SL_AS5047P

#include  "stdint.h" 

#define AS5047PMAXPOS 16384      //0x3fff
   
typedef struct Wheel
{
  float zero_position;   //上电位置
  float now_position;    //当前位置
  float last_position;   //上次位置
  float now_speed;  
  float now_angle;
  float last_angle;
  float circlesum;        //半圈记录
  int now_circlenum;
  int last_circlenum;
  float delta_distance;       //wheel移动距离
  float full_distance;       //wheel移动距离
}Wheel;  

uint16_t as5047p_Get_Position_x();
uint16_t as5047p_Get_Position_y();
void Get_Basic_x();
void Get_Basic_y();

//外部函数
void wheel_init();
void Get_Wheel_x();
void Get_Wheel_y();
void Show_Wheel_x();
void Show_Wheel_y();
//外部变量
extern Wheel wheel_x;
extern Wheel wheel_y;
extern float diameter_x;
extern float diameter_y;
#endif // SL_AS5047P
   
#ifdef __cplusplus
}
#endif
#endif /*__ AS5047P_H */