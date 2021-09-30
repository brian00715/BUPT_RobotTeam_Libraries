# SimpleLib基本配置
Author: KY_Zhang
Date: 2021-09-27
[toc]
---
## 1 STM32 CubeMX配置

> ==注意==：以下配置均基于STM32 F407ZG。如果不使用定时器，则请不要导入`/core/sys_func.c`和`sys_func.h`。

### 1.1 基本配置

* *System Core/**SYS**/Mode*
  * `Debug`: Serial Wire
* *System Core/**RCC**/Mode*
  * `HSE`: Crystal/Ceramic Resonator
* ***Clock Configuration***
  * `HSE Input frequency`: 8MHz
  * `PLL Source Mux`: HSE
  * `System Clock Mux`: PLLCLK
  * `HCLK(MHz)`: 168

### 1.2 UART配置

* *Connectivity/**USART1**/Mode*
  * `Mode`: Asynchronous
* *Connectivity/**USART1**/Configuration/NVIC Settings*
  - `USART1 global interrupt`: ✔
* *Connectivity/**USART1**/Configuration/DMA Settings*
  - 点击`Add`添加`USART1_RX`
    - `Mode`: **Circular**
  - 点击`Add`添加`USART1_TX`
* *System Core/**NVIC**/Configuration*
  * `USART1 global interrupt`
    * `Preemption Priority`: 4
  
  >  ==注意==：根据主控板PCB的外设分配情况和使用需求决定使用哪个UART。

### 1.3  CAN配置

* *Connectivity/**CAN1**/Mode*
  
  - `Activated`: ✔
* *Connectivity/**CAN1**/Configuration/Parameter Settings*
  - `Time Quanta in Bit Segment 2`: 4 Times
  - `Time Quanta in Bit Segment 1`: 9 Times
  - `Prescaler(for Time Quantum)`: 3
  - `Automatic Bus-Off Management`: Enable
  - `Transmit Fifo Priority`: Enable
* *Connectivity/CAN1/Configuration/NVIC Settings*
  
  - `CAN1 RX0 interrupts`: ✔
* *System Core/**NVIC**/Configuration*
  * `CAN1 RX0 interrupts`
    * `Preemption Priority`: 3
  
  >  ==注意==：根据主控板PCB的外设分配情况和使用需求决定使用哪个CAN。

### 1.4 项目配置

进入Project Manager

* *Project/Linker Settings*
  * `Minimum Heap Size`: 0x1000
  * `Minimum Stack Size`: 0x1200
* Code Generator/Generated files
  * `Generate peripheral initialization as a pair of '.c/.h' files per peripheral`: ✔

## 2 代码配置

* 在工程中加入所需模块的源文件(\*.c)和头文件(\*.h), 基本的模块包括：

  * cmd: 串口命令行
  * can: can总线收发管理
  * core: 标志位和其他函数 
    * (注：如果没有使用定时器不要包含sys_func.c文件)
  * utils: 哈希表，PID等常用工具

* 在main.c文件中添加代码：

  * 包含头文件：

    ```c#
    /* USER CODE BEGIN Includes */
    #include "simplelib.h"
    /* USER CODE END Includes */
    ```

  * 外设初始化完成后，进行simplelib初始化：

    ```c
    /* USER CODE BEGIN 2 */
    SimpleLib_Init(&huart1, &hcan1);
    /* USER CODE END 2 */
    ```

    > `SimpleLib_Init()`的两个参数分别是simplelib会用到的uart和can

  * 在while循环中调用simplelib处理函数：

    ```c
    /* Infinite loop */
      /* USER CODE BEGIN WHILE */
      while (1)
      {
    		SimpleLib_Run();
        /* USER CODE END WHILE */
    
        /* USER CODE BEGIN 3 */
      }
      /* USER CODE END 3 */
    ```

* 在stm32f4xx_it.c文件中添加代码：

  * 包含头文件：

    ```c
    /* USER CODE BEGIN Includes */
    #include "simplelib.h"
    /* USER CODE END Includes */
    ```

  * 串口中断服务程序中调用空闲中断处理函数：

    ```c
    void USART1_IRQHandler(void)
    {
      /* USER CODE BEGIN USART1_IRQn 0 */
    	if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
    	{
        HAL_UART_IDLECallback(&huart1);
        return ;
      }
      /* USER CODE END USART1_IRQn 0 */
      HAL_UART_IRQHandler(&huart1);
      /* USER CODE BEGIN USART1_IRQn 1 */
    
      /* USER CODE END USART1_IRQn 1 */
    }
    ```

    > 注意：添加的代码要放在`HAL_UART_IRQHandler`之前。

+ 在`cmd_func.c`的`CMD_FuncInit()`函数中注册自定义函数，例如

  ```c
  void CMD_FuncInit(void) 
  {
      CMD_Add("hello", "just", cmd_hello_func);
      // 以下为自定义代码，配置时无需添加
      CMD_Add("ChangePosMode", "", CMD_ChangePosMode);
      CMD_Add("ChangeCtrlMode", "", CMD_ChangeCtrlMode);
  }
  
  ```

+ 在can_func.c的`CAN_AcceptID_Std`数组中添加希望接收的CAN报文ID，否则将会被过滤器过滤。

+ 在`can_func.c`的`CAN_FuncInit()`函数中注册自定义回调函数，例如

  ```c
  void CAN_FuncInit()
  {
      // 以下为自定义代码，配置时无需添加
      CAN_CallbackAdd(324, CAN_Callback_Handle_Rocker);
      CAN_CallbackAdd(325, CAN_Callback_Handle_Button);
      CAN_CallbackAdd(0x281, CAN_Callback_DJI_ReadInfo);
      CAN_CallbackAdd(0x282, CAN_Callback_DJI_ReadAllPosInfo);
  }
  ```
  
  

## 3 其他配置

1. 通过在`simplelib_cfg.h`中注释/取消注释相应的宏来启用/关闭一些功能。
2. 如果要使用串口示波器，则需要在`/module/toolBoxScope.h`中修改**TOOLBOXSCOPE_UART**宏，对应希望配置的uart。
