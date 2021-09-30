/*******************************************************************************
 * Copyright:		BUPT
 * File Name:		cmd.c
 * Description:		串口命令行功能实现
 * Author:			ZeroVoid, ZX, simon, KYZhang
 * Version:			0.3
 * Data:			2021-09-27
 *******************************************************************************/

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "cmd.h"
#include "hash.h"
// TODO 分离ChassisLib和SimpleLib // #include "base_chassis.h"

/* 变量定义 -----------------------------------------------------*/
static const char *delim = ", \r\n\0";
static HashTable CMD_CommandTable; // 指令表
uint8_t UART_DMA_RxOK_Flag = 0;
UART_HandleTypeDef *slib_cmd_huart;
char *cmd_argv[MAX_ARGV];
uint8_t UART_RxBuffer_Raw[DMA_BUFFER_SIZE];
char UART_RxBuffer[DMA_BUFFER_SIZE];

// TODO 分离ChassisLib和SimpleLib // #define Locator_UART (huart6) // action全场定位接口
// static union
// {
//     uint8_t data[24];
//     float ActVal[6];
// } VegaData_u;                          //用于全场定位传输数据的结构体
// int UART_RxOK_Flag_Locator = 0;        //vega接收buffer
// char UART_RxBuffer_Locator[99];        //全场定位接收buffer
// uint8_t UART_RxBuffer_Locator_Raw[99]; //全场定位接收buffer
// void UART_DMA_Exe_Locator();           //全场定位：将读到的数据存入（结合IDLE中断使用）

/* private functions -----------------------------------------------------*/
static int str_cmp(const void *a, const void *b);
static void _cmd_help(const void *key, void **value, void *c1);

/* pubLic functions -----------------------------------------------------*/
void USART_DMA_Init(UART_HandleTypeDef *cmd_usart)
{
    slib_cmd_huart = cmd_usart;
    // 首次DMA接收，务必开启
    HAL_UART_Receive_DMA(slib_cmd_huart, (uint8_t *)&UART_RxBuffer_Raw, 99);
    // TODO 分离ChassisLib和SimpleLib
    // HAL_UART_Receive_DMA(&Locator_UART, (uint8_t *)&UART_RxBuffer_Locator_Raw, 99);
    CMD_Init();
    // TODO 分离ChassisLib和SimpleLib
    __HAL_UART_ENABLE_IT(slib_cmd_huart, UART_IT_IDLE); // 开启空闲中断
    // __HAL_UART_ENABLE_IT(&Locator_UART, UART_IT_IDLE);
}

/**
 * @brief	指令初始化函数，仅供模块初始化调用
 * @return	None
 */
void CMD_Init(void)
{
    if (CMD_CommandTable == NULL)
    {
        CMD_CommandTable = HashTable_Create(str_cmp, HashStr, NULL);
    }
    CMD_Add("help", "show cmd usage", CMD_HelpFunc);
}

void USART_DMA_Exe()
{
    int cmd_argc;
    int erro_n;
    erro_n = CMD_Parse((char *)UART_RxBuffer, &cmd_argc, cmd_argv); //解析命令
    erro_n = CMD_Exec(cmd_argc, cmd_argv);                          //执行命令
    UNUSED(erro_n);
    memset(UART_RxBuffer, 0, 98);
}

/**
 * @brief 串口中断回调函数
 * @param huart 串口号
 */
void HAL_UART_IDLECallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == slib_cmd_huart->Instance)
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart); //清除空闲标志
        // uint8_t temp;
        // temp = huart->Instance->SR;
        // temp = huart->Instance->DR; //读出串口的数据，防止在关闭DMA期间有数据进来，造成ORE错误
        // UNUSED(temp);
        /* HAL_UART_DMAStop(&CMD_UART); */ //停止本次DMA
        CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);
        HAL_DMA_Abort(huart->hdmarx);
        CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
        CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);
        huart->RxState = HAL_UART_STATE_READY;
        uint8_t *clr = UART_RxBuffer_Raw;
        while (*(clr++) == '\0' && clr < UART_RxBuffer_Raw + DMA_BUFFER_SIZE) // 找到开头，避免传输噪声
            ;
        strcpy((char *)UART_RxBuffer, (char *)(clr - 1));
        if (UART_RxBuffer[0] != '\0')
        {
            UART_DMA_RxOK_Flag = 1;
        }
        memset(UART_RxBuffer_Raw, 0, DMA_BUFFER_SIZE);
        HAL_UART_Receive_DMA(slib_cmd_huart, (uint8_t *)&UART_RxBuffer_Raw, DMA_BUFFER_SIZE); // 开启下一次中断
    }
    // TODO 分离ChassisLib和SimpleLib
    // if (huart->Instance == Locator_UART.Instance)
    // {
    //     uint8_t temp;
    //     __HAL_UART_CLEAR_IDLEFLAG(huart); //清除函数空闲标志
    //     temp = huart->Instance->SR;
    //     temp = huart->Instance->DR; //读出串口的数据，防止在关闭DMA期间有数据进来，造成ORE错误
    //     //temp = hdma_usart3_rx.Instance->CNDTR; // 获取剩余字节数
    //     HAL_UART_DMAStop(&Locator_UART); //停止本次DMA
    //     UNUSED(temp);
    //     strcpy((char *)UART_RxBuffer_Locator, (char *)UART_RxBuffer_Locator_Raw);
    //     if (UART_RxBuffer_Locator[0] != '\0')
    //         UART_RxOK_Flag_Locator = 1;

    //     UART_DMA_Exe_Locator();                                                // 将全场定位通过串口发送的消息存入
    //     memset(UART_RxBuffer_Locator_Raw, 0, 98);                                        // 缓存数组清零
    //     HAL_UART_Receive_DMA(&Locator_UART, (uint8_t *)&UART_RxBuffer_Locator_Raw, 99); //开启DMA串口中断
    // }
}

/**
 * @brief	将输入分割，并记录参数个数
 * @param	cmd_line	输入指令字符串
 * @param   argc        指令个数
 * @param   argv        分割后参数列表
 * @return	None
 */
int CMD_Parse(char *cmd_line, int *argc, char *argv[])
{
    char *token = strtok(cmd_line, delim);
    int arg_index = 0;

    while (token && arg_index <= MAX_ARGV)
    {
        argv[arg_index++] = token;
        token = strtok(NULL, delim);
    }
    *argc = arg_index;
    return 0;
}

/**
 * @brief	指令执行函数
 * @param	argc    参数个数
 * @param   argv    参数列表 
 * @return	0   正常执行返回
 *          1   未找到指令
 */
int CMD_Exec(int argc, char *argv[])
{
    struct cmd_info *cmd = (struct cmd_info *)HashTable_GetValue(CMD_CommandTable, argv[0]);
    if (cmd != NULL)
    {
        cmd->cmd_func(argc, argv);
        return 0;
    }
    uprintf("cmd not find\r\n");
    return 1;
}

/**
 * @brief	指令帮助函数
 * @param	忽略参数
 * @return	None
 */
void CMD_HelpFunc(int argc, char *argv[])
{
    // FIXME: ZeroVoid	2019/09/23	 dma usage 输出不完整，调试输出没问题
    uprintf("===============================help===============================\r\n");
    uprintf("|              CMD              |           Description          |\r\n");
    HashTable_Map(CMD_CommandTable, _cmd_help, NULL); // 遍历哈希表，打印所有帮助指令
    uprintf("==================================================================\r\n");
}

/**
 * @brief	指令添加函数
 * @param	cmd_name    指令名称
 * @param   cmd_usage   指令使用说明
 * @param   cmd_func    指令函数指针 argc 参数个数(含指令名称), argv 参数字符串数组
 * @return	None
 */
void CMD_Add(char *cmd_name, char *cmd_usage, void (*cmd_func)(int argc, char *argv[]))
{
    // FIXME: ZeroVoid	2019/9/23	 name or usage too long
    struct cmd_info *new_cmd = (struct cmd_info *)malloc(sizeof(struct cmd_info));
    char *name = (char *)malloc(sizeof(char) * (strlen(cmd_name) + 1));
    char *usage = (char *)malloc(sizeof(char) * (strlen(cmd_usage) + 1));
    strcpy(name, cmd_name);
    strcpy(usage, cmd_usage);
    new_cmd->cmd_func = cmd_func;
    new_cmd->cmd_usage = usage;
    HashTable_Insert(CMD_CommandTable, name, new_cmd);
}

char print_buffer[PRINT_BUFFER_SIZE];
void uprintf(char *fmt, ...)
{
    // 等待DMA准备完毕
    while (HAL_DMA_GetState(slib_cmd_huart->hdmatx) == HAL_DMA_STATE_BUSY)
        HAL_Delay(1);
    int size;
    va_list arg_ptr;
    va_start(arg_ptr, fmt);
    size = vsnprintf(print_buffer, PRINT_BUFFER_SIZE, fmt, arg_ptr);
    va_end(arg_ptr);

    // 重新设置USART准备状态，并解锁串口,否则无法再次输出
    // CMD_UART.gState = HAL_UART_STATE_READY;
    //__HAL_UNLOCK(&CMD_UART);
    //HAL_UART_Transmit_DMA(&CMD_UART, (uint8_t *)print_buffer, size);
    /*
    if (HAL_UART_Transmit_DMA(&CMD_UART, (uint8_t *)print_buffer, size) != HAL_OK) {
        HAL_Delay(10);
    }
    */

    // TODO:	ZeroVoid	due:10/7	优化输出，异步输出，可能纯在busy时再次调用，会被忽略，输出缺失
    // while(CMD_UART.hdmatx->State != HAL_DMA_STATE_READY);
    // HAL_UART_Transmit(&CMD_UART, (uint8_t *)print_buffer, size, 0xffff);

    HAL_UART_Transmit_DMA(slib_cmd_huart, (uint8_t *)print_buffer, size);
}

void uprintf_to(UART_HandleTypeDef *huart, char *fmt, ...)
{
    // 等待DMA准备完毕
    while (HAL_DMA_GetState(slib_cmd_huart->hdmatx) == HAL_DMA_STATE_BUSY)
        HAL_Delay(1);
    int size;
    va_list arg_ptr;
    va_start(arg_ptr, fmt);
    size = vsnprintf(print_buffer, PRINT_BUFFER_SIZE, fmt, arg_ptr);
    va_end(arg_ptr);

    HAL_UART_Transmit_DMA(huart, (uint8_t *)print_buffer, size);
    // HAL_UART_Transmit(huart,(uint8_t *)print_buffer,size,1000);
}

static char s[22] = {'b', 'y', 16, 6};
void send_wave(float arg1, float arg2, float arg3, float arg4)
{
    //s[2] = 16;  // length
    //s[3] = 6;   // type
    s[20] = '\r';
    s[21] = '\n';
    memcpy(s + 4, &arg1, sizeof(arg1));
    memcpy(s + 8, &arg2, sizeof(arg1));
    memcpy(s + 12, &arg3, sizeof(arg1));
    memcpy(s + 16, &arg4, sizeof(arg1));
    CMD_UART.gState = HAL_UART_STATE_READY;
    HAL_UART_Transmit_DMA(&CMD_UART, (uint8_t *)s, 22);
}

/* private function defined -----------------------------------------------------*/
/**
 * @brief	字符串比较函数，hash表中使用
 */
static int str_cmp(const void *a, const void *b)
{
    return strcmp((char *)a, (char *)b) != 0;
}

/**
 * @brief	输出函数使用说明，遍历hash表中使用
 * @param key cmd_name
 * @param value cmd_usage
 */
static void _cmd_help(const void *key, void **value, void *c1)
{
    UNUSED(c1);
    char *usage = ((struct cmd_info *)(*value))->cmd_usage;
    uprintf("|%31s: %-31s|\r\n", key, usage);
}

// TODO 分离ChassisLib和SimpleLib
// /**
//  * @brief 将全场定位从串口收到的数据存入
//  *        说明: 更新底盘目前位置的函数
//  *        移植者: zx
//  * @param void
//  * @return void 注：直接将结果写入了底盘结构体中
//  */
// void UART_DMA_Exe_Locator()
// {
//     if (UART_RxOK_Flag_Locator)
//     {
//         for (int j = 0; j < 99; j++)
//         {
//             if (UART_RxBuffer_Locator_Raw[j] == 0x0d) // 0x0d是回车符，0x0a是换行符
//             {
//                 if (UART_RxBuffer_Locator_Raw[j + 1] == 0x0a && j < 73 && UART_RxBuffer_Locator_Raw[j + 26] == 0x0a && UART_RxBuffer_Locator_Raw[j + 27] == 0x0d)
//                 {
//                     for (int k = 0; k < 24; k++)
//                     {
//                         VegaData_u.data[k] = UART_RxBuffer_Locator_Raw[j + 2 + k];
//                     }
// /**
//  * @note    今后最好在cmd.h中声明一个通用的全局底盘状态结构体变量，不同类型的底盘结构体只需包含此结构体的指针即可获取信息
//  */
// #ifdef NORMAL_CHASSIS
//                     chassis.vega_angle = VegaData_u.ActVal[0];
//                     chassis.vega_pos_x = VegaData_u.ActVal[3] / 1000;
//                     chassis.vega_pos_y = VegaData_u.ActVal[4] / 1000;
// #endif

//                     float x = VegaData_u.ActVal[3] / 1000;
//                     float y = VegaData_u.ActVal[4] / 1000;
//                     float yaw = VegaData_u.ActVal[0];
//                     float temp_yaw = __ANGLE2RAD(yaw + 90);
//                     if (temp_yaw < 0)
//                     {
//                         temp_yaw += 2 * PI;
//                     }
//                     BaseChassis.PostureStatus.yaw = temp_yaw;
//                     BaseChassis.PostureStatus.x = -x;
//                     BaseChassis.PostureStatus.y = -y;
//                     Chassis_UpdatePostureStatus();

//                     UART_RxOK_Flag_Locator = 0;
//                     memset(UART_RxBuffer_Locator, 0, 98);
//                     return;
//                 }
//             }
//         }
//         UART_RxOK_Flag_Locator = 0;
//         memset(UART_RxBuffer_Locator, 0, 98);
//     }
// }