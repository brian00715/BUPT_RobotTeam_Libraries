/*******************************************************************************
Copyright:      simon
File name:      vec.c
Description:    定义向量运算
Author:         Simon
Version：       1.1
Data:           2021/04/13
*******************************************************************************/
#include "vec.h"
#include "math.h"

#define VEC_PI 3.1415926535

/**向量加法*/
vec Vec_Add(vec a, vec b)
{
  vec tmp;
  tmp.x = a.x + b.x;
  tmp.y = a.y + b.y;
  return tmp;
}

/** @brief 向量点积*/
double Vec_DotProduct(vec a, vec b)
{
  return a.x * b.x + a.y * b.y;
}

/** @brief 向量取模*/
double Vec_Model(vec a)
{
  return sqrt(a.x * a.x + a.y * a.y);
}

/** @brief 生成向量*/
vec Vec_Create(float x, float y)
{
  vec tmp;
  tmp.x = x;
  tmp.y = y;
  return tmp;
}

/** @brief 向量数乘*/
vec Vec_ScalarMul(vec a, double b)
{
  a.x *= b;
  a.y *= b;
  return a;
}

/** @brief 向量顺时针旋转90度*/
vec Vec_Normal(vec a)
{
  vec tmp;
  tmp.x = a.y;
  tmp.y = -a.x;
  return Vec_ScalarMul(tmp, 1 / Vec_Model(tmp));
}

/**向量归一化*/
vec Vec_Unit(vec a)
{
  double b = Vec_Model(a);
  return Vec_ScalarMul(a, 1 / b);
}
/**向量判空*/
int Vec_IsZero(vec a)
{
  if (fabs(a.x) < 1e-6 && fabs(a.y) < 1e-6)
    return 1;
  return 0;
}

/**向量求相角*/
float Vec_GetPhaseAngle(vec a)
{
  float angle = 0;
  if (0 == a.x)
  {
    if (0 < a.y)
      angle = VEC_PI / 2;
    if (0 == a.y)
      angle = 0;
    if (0 > a.y)
      angle = -VEC_PI / 2;
  }
  else
  {
    angle = atan2f(a.y, a.x);
  }
  return angle;
}

/**
 * @brief 求两向量的相角差
 * 
 * @param b 
 * @param a 
 * @return float 
 */
float Vec_PhaseAngleSub(vec b, vec a)
{
  float a_angle = Vec_GetPhaseAngle(a);
  float b_angle = Vec_GetPhaseAngle(b);

  float angle = b_angle - a_angle;
  while (angle > VEC_PI)
  {
    angle -= VEC_PI;
  }
  while (angle <= -VEC_PI)
  {
    angle += VEC_PI;
  }
  return angle;
}