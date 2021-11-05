# ChassisLib基本配置

author: Simon
date: 2021-09-28

[toc]

---

## 1 声明定时标志位

在`main.c`的`/* USER CODE BEGIN 4 */`处添加如下代码：

```c
/* USER CODE BEGIN 4 */
static unsigned int TimeFlag_1ms = 0;
unsigned int TimeFlag_10ms = 0;
// 以下定时标志位翻转周期10ms
char TimeFlag_5ms = 0;
char TimeFlag_1s = 0;
char TimeFlag_20ms = 0;
char TimeFlag_100ms = 0;
char TimeFlag_50ms = 0;
/**
 * @brief 分频定时器
 */
void inc(void)
{
  TimeFlag_1ms++;

  if (TimeFlag_1ms % 500 == 0) // 500ms
  {
  }

  if (TimeFlag_1ms % 100 == 0) // 100ms
  {
  }

  if (TimeFlag_1ms % 50 == 0) // 50ms=20hz
  {
  }

  //10ms
  if (TimeFlag_1ms % 10 == 0)
  {
    TimeFlag_20ms = (TimeFlag_10ms % 2 == 0) ? 1 : 0;
    TimeFlag_50ms = (TimeFlag_10ms % 5 == 0) ? 1 : 0;
    TimeFlag_100ms = (TimeFlag_10ms % 10 == 0) ? 1 : 0;
    TimeFlag_1s = (TimeFlag_10ms % 100 == 0) ? 1 : 0;
    TimeFlag_10ms++;
  }

  //5ms
  if (TimeFlag_1ms % 5 == 0)
  {
    TimeFlag_5ms = 1;
  }
  else
  {
    TimeFlag_5ms = 0;
  }
}
/* USER CODE END 4 */
```

并在`main.h`的`/* USER CODE BEGIN EFP */`后添加：`void inc(void);`，在`/* USER CODE BEGIN ET */`后添加：

```c
extern char TimeFlag_5ms;
extern char TimeFlag_20ms;
extern char TimeFlag_1s;
extern char TimeFlag_100ms;
extern unsigned int TimeFlag_10ms;
extern char TimeFlag_50ms;
```

然后在`stm32f4xx_it.c`的`SysTick_Handler()`函数中添加如下代码：

```c
/* USER CODE BEGIN SysTick_IRQn 0 */
 	inc();
```

## 2 选择底盘类型

### 2.1 配置底盘参数

在`/ChassisLib/chassis_common_config.h`中通过注释/取消注释宏来选择使用哪种底盘，例如要使用舵轮底盘，则宏定义形如:

```c
// #define USE_CHASSIS_OMNI
#define USE_CHASSIS_RUDDER
// #define USE_CHASSIS_MECANUM
```

在`chassis_common_config.h`中通过宏定义配置希望使用底盘的参数，如长宽、驱动盘半径、最大线速度等等。

### 2.2 使用舵轮全向轮底盘

1. 配置舵轮电机驱动器及舵向初始化方式

   在`steer_wheel.h`中注释/取消注释相关的宏，例如使用VESC驱动器和霍尔开关，则宏定义形如：

   ```c
   /* 驱动轮使用什么驱动器----------------------------------------------*/
   #define SW_DRIVE_MTR_USE_VESC
   // #define SW_DRIVE_MTR_USE_ODRIVE
   
   /* 舵轮使用什么方式初始化--------------------------------------------*/
   #define SW_INIT_BY_HALL_SWITCH
   // #define SW_INIT_BY_ABSOLUTE_ENCODER
   ```

   1. 使用霍尔开关

      需要在CubeMX中配置四个（四轮舵轮底盘）用于获取霍尔开关信号的GPIO引脚（`input`模式，无上拉下拉），并将`User Label`配置为`HallSwitch1`、...、`HallSwitch4`，引脚编号依据霍尔开关的安装位置编排。

   2. 使用绝对式编码器

      ==TODO==

## 3 设定底盘电机驱动器CAN报文发送频率（控制频率）

根据使用的底盘类型在`main.c`中包含相应的头文件。例如使用舵轮底盘，则

```c
/* USER CODE BEGIN Includes */
#include "simplelib.h"
#include "rudder_chassis.h"
/* USER CODE END Includes */
```

在`main.c`的`inc()`函数中选择某一频率（例如20hz或50ms），添加电机驱动器CAN报文发送标志位的置位代码。以舵轮底盘为例：

```c
if (TimeFlag_1ms % 50 == 0) // 50ms=20hz
{
    RudderChassis.SteerMotors.can_send_flag = 1;
    RudderChassis.DriveMotors.can_send_flag = 1;
}
```

即舵向电机和驱动电机的电机驱动器CAN报文发送频率为20hz。

## 4 其他配置

1. 在CubeMX中配置一个用于通断继电器的GPIO引脚，并将`User Label`改为Relay。
2. 在CubeMX中配置一个用于监视底盘轨迹的uart，并在`chassis_common_config.h`中更改**CHASSIS_MONITOR_UART**宏。

## 5 可调参数

+ 偏航角控制PID参数：`YawPID`结构体

  可通过串口命令行调参，调用`cmd_func.c/CMD_SetYawPID`函数。

+ 轨迹跟踪部分：

  + 轨迹跟踪法向修正PID参数：`NormalCorrPID_x`、`NormalCorrPID_y`结构体。两个PID结构体的参数需要保持一致（当然也可以不一致，来实现x、y方向上不同程度的控制效果）。
  + 轨迹跟踪过程中到达途径点的半径阈值：**PASSED_WAYPOINT_CIRCLE_TH**，当底盘距离当前期望点的距离小于该阈值时则认为已经到达该期望点。
  + 最后3个点Go2Point的起始速度和终止速度。注意要和路径点集中的期望速度相匹配，否则最后3个点会突然加速或突然减速。

+ Go2Point部分：

  + Go2Point模式下终点锁止PID参数：`LockPID`结构体。
  + 速度规划过程中加速阶段和减速阶段所占比例，默认加速占0.1、减速占0.4。
  + 速度规划时的最大速度：由`Chassis_Plan2PointSpeed()`的`max_speed`变量决定，该变量大小和期望点与起点的距离有关。距离越远，则认为中途的最大速度可以越大。缩放系数可调。
  + 到达判断阈值：**ARRIVED_CIRCIE_TH**，值越小则越希望精准到达，但也可能带来振荡。
  + 启用原地锁定PID的距离半径阈值：**LOCK_CIRCLE_TH**，值越小则越晚开启锁止PID。

  

