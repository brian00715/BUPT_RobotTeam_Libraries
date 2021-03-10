/**
 * @file chassis_common_config.h
 * @author simon
 * @brief 底盘参数配置
 * @version 0.1
 * @date 2021-04-25
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef CHASSIS_COMMON_CONFIG_H_
#define CHASSIS_COMMON_CONFIG_H_
#include "gpio.h"
#include "usart.h"
#include "base_chassis.h"

#define CHASSIS_MONITOR_UART (huart2) // 底盘轨迹监视uart

// ===========================选择底盘类型===============================
// #define USE_CHASSIS_OMNI
#define USE_CHASSIS_RUDDER
// #define USE_CHASSIS_MECANUM

// >>>>>>>>>>>>>>>>>>>>>>>>>>横辊子全向轮参数<<<<<<<<<<<<<<<<<<<<<<<<<
#ifdef USE_CHASSIS_OMNI
//========================物理参数===========================
#define Wheel2ChassisCenter (0.439f) // (m)车轮到底盘中心的距离，用来计算自转线速度
// #define Wheel_Front2Back (0.2318596321f)   // (m)前后两轮的距离
// #define Wheel_Left2Right (0.2322096321f)   // 左右两轮的距离
// #define DRIVE_WHEEL_REDUCTION_RATIO (2.0f) // 驱动轮电机与轮子的减速比
// #define DRIVE_MOTOR_POLE_PARIS (7.0f)      // 驱动电机极对数
#define DRIVE_WHEEL_RADIUS (0.116f) // (m)驱动电机半径

//计算系数===========================
#define ShapeFactor (SW_Wheel_Front2Back / SW_Wheel_Left2Right) // 底盘形状系数，表征正方形的程度
//电机配置参数===========================
#define DRIVE_WHEEL_MAX_SPEED (1.5)  // 驱动轮最大线速度m/s
#define DRIVE_WHEEL_MIN_SPEED (0.22) // 低于此速度无法驱动
#define MAX_ROTATE_VEL (5)           // 最大自转角速度rad/s
#define MIN_ROTATE_VEL (0.47)        // 高于此速度开始自转
//其他配置===========================

#endif //USE_CHASSIS_OMNI

// >>>>>>>>>>>>>>>>>>>>>>>>>>舵轮参数（U10 Plus底盘）<<<<<<<<<<<<<<<<<<<<<<<<<
#ifdef USE_CHASSIS_RUDDER
//物理参数===========================
#define RudderChassis_Wheel2ChassisCenter (0.6222f) // (m)车轮到底盘中心的距离，用来计算自转线速度
#define RudderChassis_Front2Back (0.440f)           // (m)前后两轮的距离
#define RudderChassis_Left2Right (0.440f)           // 左右两轮的距离
#define STEER_WHEEL_REDUCTION_RATIO (3.33f)         // 2006输出轴与底盘舵轮滑轨的减速比
#define DRIVE_WHEEL_REDUCTION_RATIO (1.0f)          // 驱动轮电机与轮子的减速比
#define DRIVE_MOTOR_POLE_PARIS (20.0f)              // 驱动电机极对数
#define DRIVE_WHEEL_RADIUS (0.0434f)                // (m)驱动电机半径
//计算系数===========================
#define SW_ShapeFactor (SW_Wheel_Front2Back / SW_Wheel_Left2Right) // 底盘形状系数，表征正方形的程度
//电机配置参数===========================
#define VESC_ID_BASE (90)
#define STEER_WHEEL_MAX_POS (29.32f)
#define STEER_WHEEL_MIN_POS (-29.32f)
#define STEER_WHEEL_MAX_SPEED (1000) // 线数/ms
#define STEER_WHEEL_MIN_SPEED (300)
#define DRIVE_WHEEL_MAX_SPEED (3.0)   // 驱动轮最大线速度m/s
#define DRIVE_WHEEL_MIN_SPEED (0.15)  // 低于此速度无法驱动
#define MAX_ROTATE_VEL (12)           // 最大自转角速度rad/s
#define MIN_ROTATE_VEL (0.1)          // 高于此速度开始自转
#define MAX_HANDBRAKE_CURRENT (20.0f) // 驱动轮最大手刹电流
//其他配置===========================
#define RELAY_ACCESS GPIO_PIN_SET   // 继电器通
#define RELAY_BROKEN GPIO_PIN_RESET // 继电器断
#define SW_ARRIVE_CIRCIE (0.01)
#endif //USE_CHASSIS_RUDDER

#endif // CHASSIS_COMMON_CONFIG_H_