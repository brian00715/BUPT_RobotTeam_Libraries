/*******************************************************************************
Copyright:      Bupt
File name:      karman.c
Description:    卡尔曼滤波函数
Author:         19th
Version：       1.0
Data:           2019/11/24
*******************************************************************************/
#include "karman.h"

Kal_Struct kal_distance_L = {1, 0, 0.01, 37.1160, 0, 1};
Kal_Struct kal_distance_R = {1, 0, 0.01, 37.1160, 0, 1};
Kal_Struct kal_distance_S = {1, 0, 0.01, 37.1160, 0, 1};

Kal_Struct kal_adc_L = {1, 0, 0.01, 37.1160, 0, 1};
Kal_Struct kal_adc_R = {1, 0, 0.01, 37.1160, 0, 1};
Kal_Struct kal_adc_S = {1, 0, 0.01, 37.1160, 0, 1};

float KalMan(Kal_Struct *kal, float x)
{

  float kalman_pre; //卡尔曼的预测值
  float cov_pre;    //卡尔曼预测值的协方差
  float kg;         //增益

  kalman_pre = kal->kal_out * kal->A; //计算本次卡尔曼的预测值

  cov_pre = kal->cov * kal->A * kal->A + kal->Q;

  kg = cov_pre / (cov_pre + kal->R); //计算本次的卡尔曼增益

  kal->kal_out = kalman_pre + kg * (x - kalman_pre); //通过预测值来计算本次卡尔曼滤波后的输出

  kal->cov = (1 - kg) * cov_pre;

  return kal->kal_out;
}
