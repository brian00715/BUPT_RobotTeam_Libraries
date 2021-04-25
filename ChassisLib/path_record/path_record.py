import queue
import sys
from matplotlib import pyplot as plt
import serial
import time

if __name__ == '__main__':
    log = 0
    COM = 'COM8'  # 默认主控蓝牙串口
    BAUD_RATE = 115200
    if len(sys.argv) > 1:
        COM = sys.argv[1]
        BAUD_RATE = int(sys.argv[2])
    ser = serial.Serial(COM, BAUD_RATE, timeout=5)  # 波特率115200，超时5
    ser.flushInput()  # 清空缓冲区
    print(ser)
    if not ser.isOpen():
        ser.open()  # 打开串口
    list_max_size = 1000  # 最多同时画的点数
    now_x = []
    now_y = []
    target_x = []
    target_y = []
    pos_trans_flag = False
    list_pointer = 0  # 列表指针
    plt.axis([-5, 2, -1, 5])
    while True:
        try:
            # ser.write()
            rx = str(ser.readline(), 'utf-8')[:-2]  # 删除末尾的\r\n
            if rx != '':
                if not pos_trans_flag:
                    print(rx)
                if rx == "START":
                    pos_trans_flag = True
                    print("开始记录轨迹")
                if rx == 'END':  # STM32发送END时表明跑完一段轨迹
                    pos_trans_flag = False

                if pos_trans_flag:
                    words = rx.split(" ")
                    if(words[0] == "track:"):  # 保证协议对应
                        # print(words)
                        now_x.append(float(words[1]))
                        now_y.append(float(words[2]))
                        target_x.append(float(words[3]))
                        target_y.append(float(words[4]))
                        # plt.figure("Tracking Status")
                        plt.plot(now_x, now_y, color='red')
                        plt.plot(target_x, target_y, color='blue')
                        plt.pause(0.001)
                        if len(now_x) >= list_max_size:
                            now_x.clear()
                            now_y.clear()
                            target_x.clear()
                            target_y.clear()
                    else:  # 显示其他消息
                        print(rx)
                        pass
        except KeyboardInterrupt:
            ser.close()
            print("==退出==")
            exit(0)
