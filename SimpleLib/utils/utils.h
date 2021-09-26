#ifndef __UTILS_H
#define __UTILS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "simplelib_cfg.h"
#ifdef SL_UTILS
#include "math.h"

/**
 * @brief	宏数据处理函数
 * @note	切勿在传入带有副作用的参数.e.g. *p++
 */
#define __LIMIT(value, max)  \
	if (value > max)       \
		value = max;       \
	else if (value < -max) \
	value = -max

#define __LIMIT_FROM_TO(value, min, max) \
	{                                  \
		if ((value) > (max))           \
			value = max;               \
		if ((value) < (min))           \
			value = min;               \
	}

/**
 * @brief 计算数组和，无需指定数组类型
 * @param result 需要填入double类型，然后在宏外部使用强制类型转换
 */
#define __SUM_OF_AR(AR, N, RESULT)     \
	{                                \
		double MACRO_SUM_TEMP = 0;   \
		for (int i = 0; i < N; i++)  \
		{                            \
			MACRO_SUM_TEMP += AR[i]; \
		}                            \
		RESULT = MACRO_SUM_TEMP;    \
	}

/**
 * @brief 计算数组平均数，无需指定数组类型
 * @param result 需要填入double类型
 */
#define __AVE_OF_AR(AR, N, RESULT)     \
	{                                \
		int MACRO_SUM_TEMP = 0;      \
		for (int i = 0; i < N; i++)  \
		{                            \
			MACRO_SUM_TEMP += AR[i]; \
		}                            \
		RESULT = MACRO_SUM_TEMP / N; \
	}

/**
 * @brief 获取数组中最大的元素
 * @param MAX 结果
 */
#define __MAX_OF_AR(AR, N, MAX)       \
	{                               \
		MAX = AR[0];                \
		for (int i = 0; i < N; i++) \
		{                           \
			if (AR[i] > MAX)        \
			{                       \
				MAX = AR[i];        \
			}                       \
		}                           \
	}

#define __MIN_OF_AR(AR, N, MIN)       \
	{                               \
		MIN = AR[0];                \
		for (int i = 0; i < N; i++) \
		{                           \
			if (AR[i] > MIN)        \
			{                       \
				MIN = AR[i];        \
			}                       \
		}                           \
	}

#define Min(A, B) ((A) <= (B) ? (A) : (B))
#define Max(A, B) ((A) >= (B) ? (A) : (B))
#define PI (3.1415926535898f)
#define SL_OK 0
#define SL_ERROR 1
//角度制转化为弧度制
#define __ANGLE2RAD(x) (((x)*1.0) / 180.0f * PI)
//弧度制转换为角度制
#define __RAD2ANGLE(x) (((x)*1.0) / PI * 180.0f)

	// PID结构体，成员分别为Kp Kd Ki i last_err i_max last_d I_TIME
	typedef struct PID_t
	{
		float Kp;
		float Kd;
		float Ki;
		float int_sum;
		float last_err;
		float last_last_err; // 用于增量式PID
		float int_max;
		float last_delta_err;
		float int_duty; // 积分周期（实际上决定积分时间，即积分的快慢）
		float ctrl_max; // 控制量限幅
	} PID_t;

	typedef struct Point2D
	{
		float x;
		float y;
	} Point2D; // 二维空间点

	typedef struct Point3D
	{
		float x;
		float y;
		float z;
	} Point3D;

	float PID_GetOutput(PID_t *PID, float target, float now);
	float PID_GetIncrementOutput(PID_t *PID, float target, float now);
	void reset_PID(PID_t *s);
	void PID_init();

	float AngleLimit180(float angle);
	float AngleLimitPI(float angle);
	float AngleLimitDiff(float a, float b);
	float AngleBetweenPoints(float start_x, float start_y, float end_x, float end_y);
	void Vega_CoordinateTransform(float now[3], float now_in_target[3], float target[3]);

#endif // SL_UTILS

#ifdef __cplusplus
}
#endif

#endif /* __UTILS_H */