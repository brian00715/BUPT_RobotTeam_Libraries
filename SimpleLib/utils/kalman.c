/*******************************************************************************
Copyright:      Bupt
File name:      karman.c
Description:    �������˲�����
Author:         19th & Simon
Version:        1.0
Date:           2020/11/24
*******************************************************************************/
#include "kalman.h"

float Kalman_GetOutput(Kalman_s *kal, float now_value)
{

  float kalman_pre; //��������Ԥ��ֵ
  float cov_pre;    //������Ԥ��ֵ��Э����
  float kg;         //����

  kalman_pre = kal->kal_out * kal->A; //���㱾�ο�������Ԥ��ֵ

  cov_pre = kal->cov * kal->A * kal->A + kal->Q;

  kg = cov_pre / (cov_pre + kal->R); //���㱾�εĿ���������

  kal->kal_out = kalman_pre + kg * (now_value - kalman_pre); // ͨ��Ԥ��ֵ�����㱾�ο������˲�������

  kal->cov = (1 - kg) * cov_pre;

  return kal->kal_out;
}
