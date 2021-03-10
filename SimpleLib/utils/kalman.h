#ifndef __KALMAN_H
#define __KALMAN_H

/*Struct Area*/
typedef struct Kalman_s {
  float A;                      //一般为1
  float B;                      //一般为0
  float Q;                      //系统过程噪声的协方差
  float R;                      //测量噪声的协方差
  float kal_out;               //上一次卡尔曼的输出
  float cov;                   //上一次卡尔曼的输出的协方差
} Kalman_s;


/*Function Area*/
float Kalman_GetOutput(Kalman_s *kal,float now_value);

#endif