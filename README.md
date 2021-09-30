# 北邮机器人队程序库使用说明

Author: Simon
Date: 2021-09-27

程序库分为`ChassisLib`,`MotorLib`,`SimpleLib`三个功能库，用途分别如下：

+ SimpleLib

  1. 实现了CAN中断接收回调函数的自动管理
  2. 实现了DMA方式的串口命令行
  3. 实现了常用外设的封装
  4. 实现了常用科学计算、PID算法等工具函数

+ ChassisLib

  1. ChassisLib高度依赖MotorLib。
  2. 实现了底盘控制的OOP框架，舵轮全向轮类、横辊子全向轮类、麦克纳姆轮类均继承于基类。基类包含了描述底盘的公有成员变量（例如当前位姿、期望速度等），通用的全向轮底盘状态机、跑点算法、轨迹跟踪算法的实现。
  3. 实现了舵轮全向轮、横辊子全向轮、麦克纳姆轮底盘的运动学解算及控制。
  4. 遥控手柄的数据处理等。
  5. 一些调试工具，例如`/path_record`中实现了PC接收主控通过蓝牙串口发来的底盘坐标，通过matplotlib库绘制底盘实时轨迹与期望轨迹。

+ MotorLib

  1. 实现了VESC驱动器、大疆驱动板、大疆C610/C620电调、ODrive驱动器的CAN总线收发协议封装。

  

## 1 引用方式

+ 在STM32 CubeMX中新建一个IAR工程，将本程序库整体拷贝在工程的根目录下，并重命名为`BUPT_RobotTeam_Libraries`。

  使用时最好==删除==程序库中的`.git`文件夹，否则可能会影响工程的git操作。

  工程目录的结构看起来是这样的：

  > |-- BUPT_RobotTeam_Libraries
  >
  > ​	|-- ChassisLib
  >
  > ​	|-- MotorLib
  >
  > ​	|-- SimpleLib
  >
  > |-- Core
  >
  > |-- Drivers
  >
  > |-- EWARM
  >
  > |-- .mxproject
  >
  > |-- ***.ioc

+ 打开IAR，选择`Project->Options`(或按下Alt+F7)，点击`C/C++ Compiler`，在`Preprocessor`选项卡的`Additional include directories`窗格中添加以下内容：

  ```c
  $PROJ_DIR$/../BUPT_RobotTeam_Libraries/SimpleLib
  $PROJ_DIR$/../BUPT_RobotTeam_Libraries/SimpleLib/can
  $PROJ_DIR$/../BUPT_RobotTeam_Libraries/SimpleLib/cmd
  $PROJ_DIR$/../BUPT_RobotTeam_Libraries/SimpleLib/core
  $PROJ_DIR$/../BUPT_RobotTeam_Libraries/SimpleLib/flash
  $PROJ_DIR$/../BUPT_RobotTeam_Libraries/SimpleLib/module
  $PROJ_DIR$/../BUPT_RobotTeam_Libraries/SimpleLib/nrf
  $PROJ_DIR$/../BUPT_RobotTeam_Libraries/SimpleLib/utils
  $PROJ_DIR$/../BUPT_RobotTeam_Libraries/SimpleLib/peripherals
  $PROJ_DIR$/../BUPT_RobotTeam_Libraries/ChassisLib
  $PROJ_DIR$/../BUPT_RobotTeam_Libraries/MotorLib
  ```

+ 在左侧Workspace的`/Application/User/`内新建三个Group：`ChassisLib`,`MotorLib`,`SimpleLib`，鼠标右键Group名可以看到`Add->Add Files...`，将希望使用的功能库内的==所有程序文件==都添加到相应的Group内。

+ 如果功能库内有子文件夹，则最好为每个子文件夹也创建一个group以便于管理。当然，也可以直接添加程序文件。

+ 如果只希望单独使用某个库，例如只使用`SimpleLib`，则`ChassisLib`和`MotorLib`的Group中无需添加程序文件。

## 2 配置方式

查看功能库中的相关文档进行具体配置。

+ [SimpleLib配置文档](./SimpleLib/SimpleLib基本配置.md)
+ [MotorLib配置文档](./MotorLib/MotorLib基本配置.md)
+ [ChassisLib配置文档](./ChassisLib/ChassisLib基本配置.md)

