/***************************************************************************
 * @brief 底盘控制所需工具函数
 * @author Simon
 * @date 2020/11/06
 **************************************************************************/
#include "chassis_utils.h"


/**
 * @brief 对int16类型数组的求和
 * @param n 数组长度
 */
int16_t Sum_int16ar(int16_t ar[], int n)
{
    int count = 0;
    for (int i = 0; i < n; i++)
    {
        count += ar[i];
    }
    return count;
}

/**
 * @brief 计算int16型数组的平均值
 */
float Ave_int16ar(int16_t ar[], int n)
{
    int sum = 0;
    for (int i = 0; i < n; i++)
    {
        sum += ar[i];
    }
    return sum / n;
}

float Ave_floatar(float ar[],int n)
{
    float sum = 0;
    for (int i = 0; i < n; i++)
    {
        sum += ar[i];
    }
    return sum / n;
}


