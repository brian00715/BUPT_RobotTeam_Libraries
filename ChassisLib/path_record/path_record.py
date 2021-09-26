import queue
import sys
from matplotlib import pyplot as plt
import serial
import time

COM = 'COM13'  # 默认主控蓝牙串口
BAUD_RATE = 115200
list_max_size = 1000  # 最多同时画的点数
now_x = []
now_y = []
target_x = []
target_y = []


if __name__ == '__main__':
    if len(sys.argv) > 1:  # 命令行给定参数
        COM = sys.argv[1]
        BAUD_RATE = int(sys.argv[2])
    ser = serial.Serial(COM, BAUD_RATE, timeout=5)  # 波特率115200，超时5
    ser.flushInput()  # 清空缓冲区
    print(ser)
    if not ser.isOpen():
        ser.open()  # 打开串口

    pos_trans_flag = False
    time_start = time.time()
    while True:
        try:
            rx = str(ser.readline(), 'utf-8')[:-2]  # 删除末尾的\r\n
            if rx != '':
                # print(rx)
                if not pos_trans_flag:
                    # print(rx)
                    pass
                if rx == "START":
                    pos_trans_flag = True
                    print("开始记录轨迹")
                if rx == 'END':  # STM32发送END时表明跑完一段轨迹
                    pos_trans_flag = False

                if pos_trans_flag:
                    words = rx.split(" ")  # 根据空格分割
                    if words[0] == "track:" and len(words) >= 5:  # 保证协议对应
                        # 是否是浮点数
                        try:
                            num1 = float(words[1])
                            num2 = float(words[2])
                            num3 = float(words[3])
                            num4 = float(words[4])
                        except:
                            # print("传输有噪声")
                            continue
                        if len(now_x) == 0:
                            now_x.append(num1)
                            now_y.append(num2)
                            target_x.append(num3)
                            target_y.append(num4)
                        # 数据不同才添加, 默认速度不超过5m/s
                        elif 1e-5 <= abs(num1 - now_x[-1]) <= 5 and 1e-5 <= abs(num2 - now_y[-1]) <= 5:
                            now_x.append(num1)
                            now_y.append(num2)
                            print("nowx:%5.2f  nowy:%5.2f targetx:%5.2f targety:%5.2f\r\n",
                                  num1, num2, num3, num4)
                            target_x.append(float(words[3]))
                            target_y.append(float(words[4]))
                            plt.plot(now_x, now_y, color='red', linewidth=1)
                            plt.plot(target_x, target_y, color='blue')
                            plt.axis([-6, 6, -6, 6])
                            plt.pause(0.001)  # 0.0167 -> 60fps
                        if len(now_x) >= list_max_size:
                            now_x.clear()
                            now_y.clear()
                            target_x.clear()
                            target_y.wclear()
                #     else:  # 显示其他消息
                #         print(rx)
                #         pass

                # if(time.time()-time_start >= 10):
                #     plt.axis([-6, 6, -6, 6])
                #     plt.plot(now_x, now_y)
                #     plt.show()
        except KeyboardInterrupt:
            ser.close()
            print("==退出==")
            exit(0)
