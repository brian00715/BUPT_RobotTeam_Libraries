# STM32 部分工具整理

测试兼容HAL库版本:F4 1.24.1

目前仅整理出了串口、CAN总线中断使用。

两者均采用将中断回调的实际执行函数分别注册在相关hash表中。中断中仅作相关flag转换

## TODO:

- [ ] relay moduel
- [ ] old cmd frame (may work better)

TODO: flag管理,i2c以及一些其他通信,内存管理,小型os

## 不同开发平台目录配置

### IARSlack Theme

- `git clone https://github.com/ZeroVoid10/simplelib`到Src,Inc同级目录下

- 右击项目->选项/options->C/C++ Compiler->Preprocessor 添加新include路径![add](https://upload.cc/i1/2019/09/23/dcyNSW.png)
  - 或copy`Inc`下文件到IAR的'Inc`中
- 添加文件`Src`下所有文件

### cmake

不怎么用cmake可能说明有问题

- 编译时`-I./simpleib/Inc`添加路径

### vscode platformio 环境

- `platformio.ini`中添加`build_flags = -I 路径`
- `.c`文件放到`Src`或`src`路径下
- 目前platformio直接编译 窗口输出会有奇怪问题
- 暂不推荐使用

## 使用需添加代码说明(V0.1)

- 串口及CAN配置使用STM32Cube配置
- 使用时在main的Cube相关初始化都结束之后调用初始化函数

``` c
/**
 * @brief	初始化配置
 * @param	cmd_usart   指令通信usart句柄
 * @param   hcan        CAN通信句柄
 * @return	None
 */
void simplelib_init(UART_HandleTypeDef *cmd_usart, CAN_HandleTypeDef *hcan);

// 示例 main.c
/* USER CODE BEGIN 2 */
simplelib_init(&huart3, &hcan1);
/* USER CODE END 2 */

```

- 在`main.c`的`while(1)`中添加代码

``` c
    while (1)
  {
    simplelib_run();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
```

- 在`stm32f1xx_it.c`或`stm32f4xx_it.c`配置的中断的USART下加上代码。

``` c
/** 示例 stm32f4xx_it.c
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
  // 添加的代码
  if(__HAL_UART_GET_FLAG(&CMD_USART, UART_FLAG_IDLE) != RESET){
    HAL_UART_IDLECallback(&CMD_USART);
    return ;
  }

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
  /* USER CODE END USART3_IRQn 1 */
}

```

- 建议将cmd相关函数放到`cmd_func.c`中，将can回调函数放到`can_func.c`中，并确认在`cmd_func_init(), can_func_init()`中调用注册函数
  
``` c
void cmd_func_init(void) {
    cmd_add("hello", "just", cmd_hello_func);

    #ifdef DEBUG
    cmd_add("can_test", "test can", cmd_can_test);
    #endif
}

void can_func_init() {
    #ifdef DEBUG
    can_callback_add(1, can_suc_rx);
    #endif
}

/**
 * @brief	指令添加函数
 * @param	cmd_name    指令名称
 * @param   cmd_usage   指令使用说明
 * @param   cmd_func    指令函数指针 argc 参数个数(含指令名称), argv 参数字符串数组
 * @return	None
 */
void cmd_add(char *cmd_name, char *cmd_usage, void (*cmd_func)(int argc, char *argv[]));

/**
 * @brief	添加CAN回调函数
 * @param	id          触发回调的can id 
 * @param   callback    回调函数指针 data: can接收到数据联合体
 * @return	None
 */
void can_callback_add(const uint32_t id, void (*callback)(can_msg *data));
```
