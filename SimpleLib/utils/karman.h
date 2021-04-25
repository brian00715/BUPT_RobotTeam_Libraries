#ifndef __karman_H
#define __karman_H

/*Struct Area*/
typedef struct _kal_struct{
  float A;                      //一般为1
  float B;                      //一般为0
  float Q;                      //系统过程噪声的协方差
  float R;                      //测量噪声的协方差	
  float kal_out;               //上一次卡尔曼的输出	
  float cov;                   //上一次卡尔曼的输出的协方差	
}Kal_Struct;

/*Variable Area*/
extern Kal_Struct kal_distance_L;
extern Kal_Struct kal_distance_R;
extern Kal_Struct kal_distance_S;
extern Kal_Struct kal_adc_L;
extern Kal_Struct kal_adc_R;
extern Kal_Struct kal_adc_S;

/*Function Area*/
float KalMan(Kal_Struct *kal,float x);

#endif