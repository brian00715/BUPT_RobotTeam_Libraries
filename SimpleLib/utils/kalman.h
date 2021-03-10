#ifndef __KALMAN_H
#define __KALMAN_H

/*Struct Area*/
typedef struct Kalman_s {
  float A;                      //һ��Ϊ1
  float B;                      //һ��Ϊ0
  float Q;                      //ϵͳ����������Э����
  float R;                      //����������Э����
  float kal_out;               //��һ�ο����������
  float cov;                   //��һ�ο������������Э����
} Kalman_s;


/*Function Area*/
float Kalman_GetOutput(Kalman_s *kal,float now_value);

#endif