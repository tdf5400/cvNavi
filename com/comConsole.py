#coding=utf-8

import serial
import time
from enum import Enum


class infoClass(Enum):
    ERROR = 0  # 解析错误
    A = 1  # 查询电机电流
    BA = 2  # 查询电池电流
    BS = 3  # 查询电机转速RPM
    FF = 4  # 查询故障标识
    S = 5  # 查询编码器速度RPM
    T = 6  # 查询温度
    V = 7  # 查询电压


def parseRx(data):
    """
    解析单片机返回数值
    :param data: 字符串数据
    :return: infoClass,infoData - 信息类别,信息参数
    """
    assert isinstance(data, str), "Invalid param!"

    try:
        if data[1] is '=':  # 单字节命令
            info = data[2:].split(':')
            if data[0] is 'A':  # 查询电机电流
                if not len(info) is 2:
                    return infoClass.ERROR, 1  # 数据量错误
                # 通道1，通道2
                return infoClass.A, (float(info[0]) / 10, float(info[1]) / 10)
            elif data[0] == 'S':  # 查询编码器速度RPM
                if not len(info) is 2:
                    return infoClass.ERROR, 1  # 数据量错误
                # 编码器1，编码器2
                return infoClass.S, (float(info[0]), float(info[1]))
            elif data[0] == 'T':  # 查询温度
                if not len(info) is 3:
                    return infoClass.ERROR, 1  # 数据量错误
                # 内部IC，通道1，通道3
                return infoClass.T, (float(info[0]), float(info[1]), float(info[2]))
            elif data[0] == 'V':  # 查询电压
                if not len(info) is 3:
                    return infoClass.ERROR, 1  # 数据量错误
                # 内部电压，主电池电压，输出端电压
                return infoClass.V, (float(info[0]), float(info[1]), float(info[2]))
            else:
                return infoClass.ERROR, 2  # 无效索引
        elif data[2] is '=':  # 双字符命令
            info = data[3:].split(':')
            if data[0:2] == 'BA':  # 查询电池电流
                if not len(info) == 2:
                    return infoClass.ERROR, 1  # 数据量错误
                # 通道1，通道2
                return infoClass.BA, (float(info[0]), float(info[1]))
            elif data[0:2] == 'BS':  # 查询电机转速RPM
                if not len(info) == 2:
                    return infoClass.ERROR, 1  # 数据量错误
                # 每分钟转速
                return infoClass.BS, (float(info[0]), float(info[1]))
            elif data[0:2] == 'FF':  # 查询故障标识
                if not len(info) == 1:
                    return infoClass.ERROR, 1  # 数据量错误
                # 故障号，
                # value为8bit,   bit0:过热，bit1:过压，bit2:欠压，bit3:短路
                #                bit4:紧急停止，bit5:励磁故障，bit6:MOSFET故障，bit7:启动配置故障
                return infoClass.FF, (float(info[0]))
            else:
                print(data[1:3], data[1:3] == 'BA')
                return infoClass.ERROR, 2  # 无效索引
        else:
            return infoClass.ERROR, 0
    except Exception:
        return infoClass.ERROR, 0


def getPortList():
    """
    获取串口列表
    :return: 串口列表
    """
    import serial.tools.list_ports
    port_list = list(serial.tools.list_ports.comports())
    # 输出
    if len(port_list) == 0:
        print('[Serial]\tNo Port!')
        return None
    else:
        print('[Serial]\tPorts:')
        for i in range(0, len(port_list)):
            print("[Serial]\t{}".format(port_list[i]))
        return port_list


class serialConsole:
    myPort = None  # 串口对象
    waitTime = 1  # 串口间隔时间(ms)

    def __init__(self, portx, bps, timeout, waitTime):
        """
        初始化
        :param portx:   端口号
        :param bps:     波特率
        :param timeout: 超时
        :param waitTime: 指令间隔时间(ms)
        """
        assert (isinstance(portx, str) and isinstance(bps, int)
                and isinstance(timeout, int)), "Invalid param!"
        self.myPort = serial.Serial(portx, bps, timeout=timeout)
        self.waitTime = waitTime

    def getInfo(self, getClass):
        assert isinstance(getClass, infoClass), "Invalid param!"

        self.myPort.flushInput()    # 清空缓存区
        if getClass is infoClass.A:     # 查询电机电流
            self.myPort.write('?A\r'.encode('utf-8'))
        elif getClass is infoClass.BA:  # 查询电池电流
            self.myPort.write('?BA\r'.encode('utf-8'))
        elif getClass is infoClass.BS:  # 查询电机转速RPM
            self.myPort.write('?BS\r'.encode('utf-8'))
        elif getClass is infoClass.FF:  # 查询故障标识
            self.myPort.write('?FF\r'.encode('utf-8'))
        elif getClass is infoClass.S:  # 查询编码器速度RPM
            self.myPort.write('?S\r'.encode('utf-8'))
        elif getClass is infoClass.T:  # 查询温度                
            self.myPort.write('?T\r'.encode('utf-8'))
        elif getClass is infoClass.V:  # 查询电压
            self.myPort.write('?V\r'.encode('utf-8'))

        error = 0
        while error < 6:
            time.sleep(self.waitTime)   # 等待响应
            rxData = bytes.fromhex(bytes.hex(self.myPort.readline())).decode()  # 读一行数据
            #print(rxData.split('\r'))
            for i in rxData.split('\r'):
                if parseRx(i)[0] is getClass:
                    return parseRx(i)
            error += 1
        return infoClass.ERROR, -1

    def setSta(self, sta=False):
        """
        启动/关闭指令
        :param sta: True - 开启， False - 关闭
        :return:
        """
        assert isinstance(sta, bool), "Invalid param!"
        if sta:
            self.myPort.write('!RUN=1\r'.encode('utf-8'))
        else:
            self.myPort.write('!RUN=0\r'.encode('utf-8'))
            

    def emergencySTOP(self, enable=True):
        """
        紧急停止
        :param enable: True - 停止， False - 释放
        :return:
        """
        assert isinstance(enable, bool), "Invalid param!"
        if enable:
            self.myPort.write('!EX\r'.encode('utf-8'))
        else:
            self.myPort.write('!EG\r'.encode('utf-8'))

    def setMotor(self, motor1, motor2):
        """
        电机控制函数
        None参数为不设定
        :param motor1: 电机1速度设置(-1000~1000)
        :param motor2: 电机2速度设置(-1000~1000)
        """
        if motor1 > 1000:
            motor1 = 1000
        elif motor1 < -1000:
            motor1 = -1000
        if motor2 > 1000:
            motor2 = 1000
        elif motor2 < -1000:
            motor2 = -1000
        motor1 = int(motor1)
        motor2 = int(motor2)
        self.myPort.write('!M={} {}\r'.format(motor1, motor2).encode('utf-8'))
        print('!M={} {}'.format(motor1, motor2).encode('utf-8'))


class keyBoardControl(serialConsole):
    def keyEvent(self, key):
        """
        按键任务分析
        :param key: 输入键值
        :return:
        """
        if key is 'q':  # 结束任务
            print('[kbControl]\tTask quit!')
            self.setMotor(motor1=0, motor2=0)
            return False
        elif key is '':  # 停车
            print('[kbControl]\tStop!')
            self.setMotor(motor1=0, motor2=0)
        elif key is 'w':  # 前进
            print('[kbControl]\tAdvance!')
            self.setMotor(motor1=-100, motor2=-100)
        elif key is 's':  # 后退
            print('[kbControl]\tBack!')
            self.setMotor(motor1=100, motor2=100)
        elif key is 'a':  # 左转
            print('[kbControl]\tLeft!')
            self.setMotor(motor1=-100, motor2=100)
        elif key is 'd':  # 右转
            print('[kbControl]\tRight!')
            self.setMotor(motor1=100, motor2=-100)
        elif key is 'z':
            self.emergencySTOP(True)
            print('[kbControl]\temergencySTOP!')
        elif key is 'x':
            self.emergencySTOP(False)
            print('[kbControl]\temergencySTOP release!')
        elif key is 'c':
            info = self.getInfo(infoClass.BS)
            print('[kbControl]\tRead: {}'.format(info))
        elif key is 'v':
            info = self.getInfo(infoClass.T)
            print('[kbControl]\tRead: {}'.format(info))
        else:
            print('[kbControl]\tInvalid param!')

    def run(self):
        self.setSta(True)           # 启用32串口
        # self.emergencySTOP(False)   # 释放紧急停止
        print('[kbControl]\tTask start!')
        while True:
            try:
                key = input('[kbControl]\tplease enter cmd: ')
                if self.keyEvent(key) is False:
                    break
            except KeyboardInterrupt:  # Ctrl+C
                print('[kbControl]\tStop!')
                self.setMotor(motor1=0, motor2=0)
                print('[kbControl]\tTask quit!')
                break


if __name__ == "__main__":
    # getPortList()
    keyBoard = keyBoardControl(portx='COM10', bps=115200, timeout=1, waitTime=0.05)
    # keyBoard = keyBoardControl(portx='/dev/ttyUSB0', bps=115200, timeout=1, waitTime=0.1)
    keyBoard.run()
#    kb = parseRx('?BABA=0:00')
#    print(kb)



