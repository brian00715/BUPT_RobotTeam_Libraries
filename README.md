# 北邮机器人队
# 程序库使用说明

##### Author: Simon
##### Date: 2021/04/25

## 1. 引用方式
+ 新建一个IAR工程，将本文件夹整体拷贝在根目录下。
+ 打开IAR，选择`Project->Options`(或按下Alt+F7)，点击`C/C++ Compiler`，在`Preprocessor`选项卡的`Additional include directories`窗格中添加以下内容：

```C
$PROJ_DIR$/../BUPT_Libraries/SimpleLib
$PROJ_DIR$/../BUPT_Libraries/SimpleLib/can
$PROJ_DIR$/../BUPT_Libraries/SimpleLib/cmd
$PROJ_DIR$/../BUPT_Libraries/SimpleLib/core
$PROJ_DIR$/../BUPT_Libraries/SimpleLib/flash
$PROJ_DIR$/../BUPT_Libraries/SimpleLib/module
$PROJ_DIR$/../BUPT_Libraries/SimpleLib/nrf
$PROJ_DIR$/../BUPT_Libraries/SimpleLib/utils
$PROJ_DIR$/../BUPT_Libraries/SimpleLib/peripherals
$PROJ_DIR$/../BUPT_Libraries/ChassisLib
$PROJ_DIR$/../BUPT_Libraries/MotorLib
```
+ 在左侧Workspace的User内新建三个Group：`ChassisLib`,`MotorLib`,`SimpleLib`，并将每个库下的所有程序文件都添加到相应的Group内。例如，`SimpleLib`中包含`cmd`文件夹，则将`cmd`下的所有文件也添加到SimpleLib Group内。
+ 在`main.c`的合适位置添加如下内容：
```C
Clock clock = {0};
void clock_exe()
{
  clock.sec += clock.m_sec / 100;
  clock.min += clock.sec / 60;
  clock.sec %= 60;
  clock.m_sec %= 100;
}
// >>>计时标志�?<<< 在main.h全局公开
static unsigned int TimeFlag_1ms = 0; // int型的都为累加量，char型的为标志量
unsigned int TimeFlag_10ms = 0;
char TimeFlag_5ms = 0;
char TimeFlag_20ms = 0;
char TimeFlag_1s = 0;
char TimeFlag_100ms = 0;
char TimeFlag_50ms = 0;
char vega_is_ready = 0;
/**
 * @brief 滴答时钟，随系统时钟中断�??起执行，周期1ms
 */
void inc(void)
{
  TimeFlag_1ms++;

  //1000ms
  if (TimeFlag_1ms % 1000 == 0)
  {
  }

  //500ms
  if (TimeFlag_1ms % 500 == 0)
  {
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin); //led闪烁；由于随系统时钟中断运行，可以用以监控是否卡�??
  }

  //10ms
  if (TimeFlag_1ms % 10 == 0)
  {
    clock.m_sec++;

    TimeFlag_10ms++;
    TimeFlag_20ms = (TimeFlag_10ms % 2 == 0) ? 1 : 0; // 标志位持续时�??10ms
    TimeFlag_50ms = (TimeFlag_10ms % 5 == 0) ? 1 : 0;
    TimeFlag_100ms = (TimeFlag_10ms % 10 == 0) ? 1 : 0;
    TimeFlag_1s = (TimeFlag_10ms % 100 == 0) ? 1 : 0;
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

  //vega（全场定位）初始化时间设定，要有15s的启动时�??
  if (TimeFlag_1ms % 15000 == 0 && vega_is_ready == 0)
  {
    vega_is_ready = 1;
    uprintf("==Vega Init Done==\r\n");
  }
}
```
+ 打开`stm32fxxx_it.c`文件，在你定义的CMD串口中断服务函数以及东大全场定位串口中断服务函数中添加`HAL_UART_IDLECallback(&CMD_UART);`
在`SysTick_Handler`中添加`inc();`