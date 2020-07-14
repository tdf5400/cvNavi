# encoding:utf-8
import cv2
import numpy as np
import com.comConsole as comConsole
import navigation.roadCal as rc
from segment.floodfill import *
import time
import os

SPEED = 70      # 前进速度（0-1000）
SPEEDMAX = 120

CAP_SWITCH = 1      # 摄像头选择(0-不使用摄像头, 其他-摄像头编号+1)
SERIAL_SWITCH = 1   # 串口控制开关
DISPLAY_SWITCH = 1  # 显示处理结果
STEP_RUN = 0        # 按步运行（输入w才进入下一步动作）
path = "./o2.jpg"
PicNum = 64  # startNum when catch picture


def autoCtrl():
    if SERIAL_SWITCH:   # 串口
        #car = comConsole.serialConsole(portx='COM3', bps=115200, timeout=1, waitTime=0.05)
        car = comConsole.serialConsole(portx='/dev/ttyUSB0', bps=115200, timeout=1, waitTime=0.05)
        car.setSta(True)
        time.sleep(0.01)
        car.setSta(True)
    if CAP_SWITCH:
        camera = cv2.VideoCapture(CAP_SWITCH - 1)
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)


    while True:
        # 开始计时
        time_start = time.perf_counter()

        # 获取图像
        if CAP_SWITCH:
            # camera = cv2.VideoCapture(CAP_SWITCH - 1)
            ret, src = camera.read()
            # camera.release()
        else:
            src = cv2.imread(path)
        if src is None:  # 判断图像存在性
            print(f'[console]No Image!')
            continue

        img = cv2.resize(src, (640, 480))  # 分辨率重定义
        # copyImg, threImg = rc.cal_floodFill(img, (20, 100, 255), (40, 150, 255))  # FloodFill计算
        copyImg, threImg = floodFill(img, (66, 111, 132), (70, 130, 114), mask_wide=0)
        if threImg is None:  # 取色失败则进入下一帧
            print(f'[console]FloodFill Error!')
            state, staInfo = rc.FIT_CROSS_ERR, 0
        else:    
            state, staInfo = rc.fitRoad_cross(threImg, 30, scanPercent=0.6)
     
        if state == rc.FIT_CROSS_STRAIGHT:
            print(f'[console]Straight!', end='\t')
            print('Theta:', staInfo)
        elif state == rc.FIT_CROSS_TRUN:
            print(f'[console]Turn!', end='\t')
            print('Theta: ', staInfo)
        elif state == rc.FIT_CROSS_OUT:
            print(f'[console]Out of Road!')
            print('info:',int(staInfo*100), '%')
        else:
            print(f'[console]Error!', end='\t')
            print('Info:', staInfo)

        # 结果显示
        if DISPLAY_SWITCH and state!=rc.FIT_CROSS_ERR:
            #cv2.imshow('raw', img)
            #cv2.imshow('floodFill', copyImg)
            cv2.imshow('Threshold', threImg)
            if state == rc.FIT_CROSS_STRAIGHT or state == rc.FIT_CROSS_TRUN:
                directImg = copyImg.copy()
                img_h, img_w = directImg.shape[:2]
                line_point = (int(img_w / 2 - 1 - img_h * staInfo), 0)
                cv2.line(directImg, (int(img_w / 2 - 1), int(img_h - 1)), line_point, (0, 0, 255), 3)
                cv2.imshow('Direct', directImg)
            else:
                cv2.imshow('Direct', img)

        # 显示处理时间
        time_elapsed = (time.perf_counter() - time_start)
        print(f'[console]Used:\t{int(time_elapsed * 1000)} ms', )
        # print("[console]Fre:\t%0.2f" % (1 / time_elapsed), " Hz")

        # 串口输出
        if SERIAL_SWITCH:
            # 斜率转电机转速
            # print(f'staInfo={staInfo}')
            if state == rc.FIT_CROSS_STRAIGHT:  # 直
                P = 120
                motor1 = SPEED + (P*staInfo)
                motor2 = SPEED - (P*staInfo)
                if motor1 > SPEEDMAX:
                    motor1 = SPEEDMAX
                elif motor1 < -SPEEDMAX:
                    motor1 = -SPEEDMAX
                if motor2 > SPEEDMAX:
                    motor2 = SPEEDMAX
                elif motor2 < -SPEEDMAX:
                    motor2 = -SPEEDMAX
                #print(motor1, motor2)
                car.setMotor(-motor1, -motor2)
                #if staInfo > 0:
                #    car.setMotor(-(120), -(15))
                #else:
                #    car.setMotor(-(15), -(120))
            elif state == rc.FIT_CROSS_TRUN:    # 弯
                P = 53
                motor1 = SPEED + P*staInfo
                motor2 = SPEED - P*staInfo
                if motor1 > SPEEDMAX:
                    motor1 = SPEEDMAX
                elif motor1 < -SPEEDMAX:
                    motor1 = -SPEEDMAX
                if motor2 > SPEEDMAX:
                    motor2 = SPEEDMAX
                elif motor2 < -SPEEDMAX:
                    motor2 = -SPEEDMAX
                #print(motor1, motor2)
                car.setMotor(-motor1, -motor2)
                #if staInfo >0:
                #    car.setMotor(-(100), -(-100))
                #else:
                #    car.setMotor(-(-100), -(100))
            elif state == rc.FIT_CROSS_OUT:     # 出
                if staInfo > 0.2:
                    motor1 = int(SPEED/2)
                    motor2 = int(SPEED/2)
                else:
                    motor1 = 0
                    motor2 = 0
                #print(motor1, motor2)
                car.setMotor(-motor1, -motor2)
            else:
                car.setMotor(0, 0)

        # Esc退出
        if STEP_RUN:
            time.sleep(1)
            car.setMotor(0, 0)
            keyAction = cv2.waitKey(10)
            while keyAction != 119:     # 等待w
                keyAction = cv2.waitKey(10)   # 等待按键
                if keyAction == 27:     # Esc
                    if SERIAL_SWITCH:
                        car.setMotor(0, 0)
                    cv2.destroyAllWindows()
                    return 1
                elif keyAction == 13:     # enter切换是否启动
                    if SERIAL_SWITCH:
                        car.setMotor(0, 0)
                        while cv2.waitKey() != 13:
                            pass
        else:
            keyAction = cv2.waitKey(100)  # 延时1ms
            if keyAction == 27:     # Esc
                if SERIAL_SWITCH:
                    car.setMotor(0, 0)
                cv2.destroyAllWindows()
                return 1
            elif keyAction == 13:  # enter切换是否启动
                if SERIAL_SWITCH:
                    car.setMotor(0, 0)
                    while cv2.waitKey() != 13:
                        pass
            elif keyAction == 115:  # savePicture
                try:
                    global PicNum
                    path = '/home/pi/Desktop/cvNavi/camera/'
                    if not os.path.exists(path):
                        os.makedirs(path)
                    cv2.imwrite(path+'{}.jpg'.format(PicNum), src)
                    print('[console]\tsave pic succeed!')
                    PicNum += 1
                except Exception:
                    print('[console]\tsave pic failed!')

def keyCtrl():
    comConsole.getPortList()
    #kb = comConsole.keyBoardControl(portx='COM3', bps=115200, timeout=1, waitTime=0.05)
    kb = comConsole.keyBoardControl(portx='/dev/ttyUSB0', bps=115200, timeout=1, waitTime=0.05)
    kb.setSta(True)
    kb.run()

if __name__ == "__main__":
    while True:
        key = input('Mode 1: autoCtrl\nMode 2: keyCtrl\n')
        # key = '1'
        if key == '1':
            autoCtrl()
        elif key == '2':
            keyCtrl()
        else:
            print('Invalid param')

