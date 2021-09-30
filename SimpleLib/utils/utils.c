#include "utils.h"

void PID_init(PID_s *PID)
{
}

/**
 * @brief 位置式PID
 * 
 * @param PID 
 * @param target 
 * @param now 
 * @return float 
 */
float PID_GetOutput(PID_s *PID, float target, float now)
{
  float err;
  float delta_err;
  float result;

  err = target - now;
  delta_err = err - PID->last_err;

  delta_err *= 0.384f;
  delta_err += PID->last_delta_err * 0.615f; //低通滤波

  PID->last_err = err;

  PID->int_sum += err * PID->int_duty; // 积分量

  __LIMIT(PID->int_sum, PID->int_max); // 限制积分量大小
  PID->last_delta_err = delta_err;

  result = err * PID->Kp + delta_err * PID->Kd + PID->int_sum * PID->Ki;
  __LIMIT(result, PID->ctrl_max);
  return result;
}

/**
 * @brief 增量式PID
 * 
 * @param PID 
 * @param target 
 * @param now 
 * @return float 
 */
float PID_GetIncrementOutput(PID_s *PID, float target, float now)
{
  float err = target - now;
  float delta = PID->Kp * (err - PID->last_err) +
                PID->Ki * err +
                PID->Kd * (err - 2 * PID->last_err + PID->last_last_err);
  PID->last_last_err = PID->last_err;
  PID->last_err = err;
  __LIMIT(delta,PID->ctrl_max);
  return delta;
}

void PID_Reset(PID_s *s)
{
  s->int_sum = 0;
  s->last_err = 0;
  s->last_delta_err = 0;
}

float AngleLimit180(float angle)
{
  while (angle > 180)
  {
    angle -= 360;
  }
  while (angle <= -180)
  {
    angle += 360;
  }
  return angle;
}

float AngleLimitPI(float angle)
{
  while (angle > PI)
  {
    angle -= 2 * PI;
  }
  while (angle <= -PI)
  {
    angle += 2 * PI;
  }
  return angle;
}

/**
 * @brief 两角差值，限制在[0,pi]
 * 
 */
float AngleLimitDiff(float a, float b)
{
  float out = a - b;
  return AngleLimitPI(out);
}

float AngleBetweenPoints(float start_x, float start_y, float end_x, float end_y)
{
  float angle = 0;
  if (fabs(start_x - end_x) < 1e-4) // 浮点数不能直接比较大小
  {
    if (fabs(start_y - end_y) < 1e-4)
      angle = 0;
    else if (start_y < end_y)
      angle = PI / 2;
    else //(start_y > end_y)
      angle = -PI / 2;
  }
  else
  {
    angle = atan2f(end_y - start_y, end_x - start_x);
  }
  return angle;
}