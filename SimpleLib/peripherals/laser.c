/*******************************************************************************
Copyright:      Bupt
File name:      laser.c
Description:    激光
Author:         20th
Version：       1.0
Data:           2019/12/9
*******************************************************************************/
#include "laser.h"
#include "cmd.h"
#include "base_chassis.h"
#ifdef USE_LASER
//TODO :区分三个激光分别对应adc的第几个通道

#define ERROR_ON_LEFT 0 //三个激光与实际的偏差
#define ERROR_ON_RIGHT 0
#define ERROR_ON_SIDE 0
#define DIS_BTW_LR 0.45

float DISTANCE_LASERHL_AND_LASERHR = 0.279; // 单位m

LASER laser_left;
LASER laser_side;
LASER laser_right;

LASER_ADC laser_adc[AVERAGE_AMOUNT]; // 直接从端口读入的原始数据

PID_s laser_ypos_pid = {200, 50, 0, 0, 0, 0, 0, 0.005};
PID_s laser_xpos_pid = {700, 50, 0, 0, 0, 0, 0, 0.005};

/**激光线性方程参数*/
void laser_calculate_kb(LASER *sensor)
{
  sensor->k_param = (sensor->FAR_distance - sensor->NEAR_distance) / (sensor->FAR_voltage - sensor->NEAR_voltage);
  sensor->b_param = sensor->FAR_distance - sensor->k_param * sensor->FAR_voltage;
  // uprintf("k is %f and b is %f\r\n", sensor->k_param, sensor->b_param);
}

/**激光ADC三个通道分别取数*/
void laser_adc_split(LASER *laser_l, LASER *laser_r, LASER *laser_s)
{

  for (int i = 0; i < AVERAGE_AMOUNT; i++)
  {
    laser_l->ADC_value[i] = laser_adc[i].adc_l;
    laser_r->ADC_value[i] = laser_adc[i].adc_r;
    laser_s->ADC_value[i] = laser_adc[i].adc_s;
  }
}

/**激光计算距离*/
float laser_calculate_distance(LASER *sensor, Kal_Struct *kal_laser_distance, Kal_Struct *kal_laser_adc)
{
  float sum_up = 0;
  float distance;
  for (int i = 0; i < AVERAGE_AMOUNT; i++)
  {
    sum_up += sensor->ADC_value[i];
  }
  sensor->ADC_final = (int)sum_up / AVERAGE_AMOUNT; // 均值滤波
  sensor->ADC_final = (int)KalMan(kal_laser_adc, sensor->ADC_final);
  distance = sensor->ADC_final * sensor->k_param + sensor->b_param;
  distance = KalMan(kal_laser_distance, distance);
  return distance;
}

//激光计算x  y  angle 的方式
/**激光计算x*/
float laser_calculate_x()
{
  float laser_x = laser_side.distance;
  return laser_x;
}
/**激光计算y*/
float laser_calculate_y()
{
  float laser_y = 0;
  return laser_y;
}

/**激光计算角度*/
float laser_calculate_angle()
{
  // float laser_offset = laser_right.distance - laser_left.distance;
  // float laser_angle = atan(laser_offset / DISTANCE_LASERHL_AND_LASERHR);
  // return laser_angle;
  // 2020/10/17 根据新的挡板，跑准x和y后偏航角也就跑好了
  return 0; 
}

/**激光初始化:计算kb;打开DMA*/
void laser_init()
{

  //TODO 待校准
  laser_left.FAR_distance = 0.7;
  laser_left.FAR_voltage = 3627;
  laser_left.NEAR_distance = 0.2;
  laser_left.NEAR_voltage = 381;

  laser_right.NEAR_voltage = 576;
  laser_right.NEAR_distance = 0.20;
  laser_right.FAR_distance = 0.70;
  laser_right.FAR_voltage = 3069;

  laser_side.NEAR_voltage = 414;
  laser_side.NEAR_distance = 0.20;
  laser_side.FAR_distance = 1.10;
  laser_side.FAR_voltage = 3762;

  // uprintf("Left---");
  laser_calculate_kb(&laser_left);
  // uprintf("Right---");
  laser_calculate_kb(&laser_right);
  // uprintf("Side---");
  laser_calculate_kb(&laser_side);

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)laser_adc, 3 * AVERAGE_AMOUNT) != HAL_OK)
  {
    uprintf("\r\n##Laser DMA Wrong!##\r\n");
    while (1)
    {
    }
  }
}

/**激光执行函数: 获取三个激光的距离;计算x y angle*/
void laser_exe()
{
  laser_adc_split(&laser_left, &laser_right, &laser_side);
  // left和right分别时左后和右后的激光
  laser_left.distance = laser_calculate_distance(&laser_left, &kal_distance_L, &kal_adc_L) + ERROR_ON_LEFT;
  laser_right.distance = laser_calculate_distance(&laser_right, &kal_distance_R, &kal_adc_R) + ERROR_ON_RIGHT;
  // side是右边的激光
  laser_side.distance = laser_calculate_distance(&laser_side, &kal_distance_S, &kal_adc_S) + ERROR_ON_SIDE;

  // 坐标系定义为水平向左为x轴正方向，垂直向上为y轴正方向
  BaseChassis.PostureStatus.laser_pos_yaw = laser_calculate_angle(); // 需要首先计算偏航角以供x和y换算，是与x轴的夹角
  BaseChassis.PostureStatus.laser_pos_x = laser_calculate_x();
  BaseChassis.PostureStatus.laser_pos_y = laser_calculate_y();

}

/**激光打印三个距离*/
void laser_print_distance()
{
  uprintf("Left :%6fm\r\n", laser_left.distance);
  uprintf("Right:%6fm\r\n", laser_right.distance);
  uprintf("Side :%6fm\r\n", laser_side.distance);
}

int Laser_PrintPos_Flag = 0;
/**激光打印pos*/
void Laser_PrintPos()
{
  if (!Laser_PrintPos_Flag)
  {
    return;
  }
  // uprintf("--Laser pos:\r\n");
  uprintf("  Laser_X:%6fm", BaseChassis.PostureStatus.laser_pos_x);
  uprintf("  Laser_Y:%6fm", BaseChassis.PostureStatus.laser_pos_y);
  uprintf("  Laser_Angle:%6f\r\n", BaseChassis.PostureStatus.laser_pos_yaw);
}

int Laser_PrintADCValue_Flag = 0;
void Laser_PrintADCValue()
{
  if (!Laser_PrintADCValue_Flag)
  {
    return;
  }
  uprintf("  left:");
  uprintf("%d ", laser_left.ADC_final);
  // uprintf("\r\n");
  uprintf("  right:");
  uprintf("%d ", laser_right.ADC_final);
  uprintf("  side:");
  uprintf("%d ", laser_side.ADC_final);
  uprintf("\r\n");
  // uprintf("--adc1 value: %d \r\n", HAL_ADC_GetValue(&hadc1));
}

#endif // USE_LASER