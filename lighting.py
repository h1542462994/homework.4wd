# -*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time
from threading import Thread

#RGB三色灯引脚定义
LED_R = 22
LED_G = 27
LED_B = 24

r_light = 0.05
r_dark = 0
g_light = 0.05
g_dark = 0
b_light = 0.05
b_dark = 0.05

threading = False
r_thread = None
g_thread = None
b_thread = None


time_light = 0.05

#设置RGB三色灯为BCM编码方式
GPIO.setmode(GPIO.BCM)
#RGB三色灯设置为输出模式
GPIO.setup(LED_R, GPIO.OUT)
GPIO.setup(LED_G, GPIO.OUT)
GPIO.setup(LED_B, GPIO.OUT)

class R_Thread(Thread):
    def __init__(self):
        # 设置RGB三色灯为BCM编码方式
        super(R_Thread, self).__init__()
        GPIO.setmode(GPIO.BCM)
        # RGB三色灯设置为输出模式
        GPIO.setup(LED_R, GPIO.OUT)
        GPIO.setup(LED_G, GPIO.OUT)
        GPIO.setup(LED_B, GPIO.OUT)
    def run(self):
        self.__init__()
        while True:
            if threading:
                GPIO.output(LED_R, GPIO.HIGH)
                time.sleep(r_light)
                GPIO.output(LED_R, GPIO.LOW)
                time.sleep(r_dark)
            else:
                GPIO.output(LED_R, GPIO.LOW)
                time.sleep(time_light)

class G_Thread(Thread):
    def __init__(self):
        # 设置RGB三色灯为BCM编码方式
        super(G_Thread, self).__init__()
        GPIO.setmode(GPIO.BCM)
        # RGB三色灯设置为输出模式
        GPIO.setup(LED_R, GPIO.OUT)
        GPIO.setup(LED_G, GPIO.OUT)
        GPIO.setup(LED_B, GPIO.OUT)
    def run(self):
        self.__init__()
        while True:
            if threading:
                GPIO.output(LED_G, GPIO.HIGH)
                time.sleep(g_light)
                GPIO.output(LED_G, GPIO.LOW)
                time.sleep(g_dark)
            else:
                GPIO.output(LED_G, GPIO.LOW)
                time.sleep(time_light)

class B_Thread(Thread):

    def __init__(self):

        # 设置RGB三色灯为BCM编码方式
        super(B_Thread, self).__init__()
        GPIO.setmode(GPIO.BCM)
        # RGB三色灯设置为输出模式
        GPIO.setup(LED_R, GPIO.OUT)
        GPIO.setup(LED_G, GPIO.OUT)
        GPIO.setup(LED_B, GPIO.OUT)

    def run(self):
        self.__init__()
        while True:
            if threading:
                GPIO.output(LED_B, GPIO.HIGH)
                time.sleep(b_light)
                GPIO.output(LED_B, GPIO.LOW)
                time.sleep(b_light)
            else:
                GPIO.output(LED_B, GPIO.LOW)
                time.sleep(time_light)

def init():
    r_thread = R_Thread()
    g_thread = G_Thread()
    b_thread = B_Thread()
    r_thread.start()
    g_thread.start()
    b_thread.start()


def start(r, g, b):
    r_light = time_light * r / 255
    r_dark = time_light - r_light
    g_light = time_light * g / 255
    g_dark = time_light - g_light
    b_light = time_light * b / 255
    b_dark = time_light - b_light
    threading = True


def stop():
    threading = False


def r_doing():
    # 从0,0,0到255,255,0过渡
    r = 255
    g = 255
    b = 0
    while True:
        r = r - 10
        g = g - 10
        start(r, g, b)
        time.sleep(1)


def doing():
    #循环显示7种不同的颜色
    try:
        while True:
            t2 = time.time() % 10
            t3 = time.time() % 11
            t4 = time.time() % 12
            r = 255 * t2 / 10
            g = 255 * t3 / 11
            b = 255 * t4 / 12

            t = time.time() % 0.05 * 1000
            if t < r / 255 * 50:
                GPIO.output(LED_R, GPIO.HIGH)
            else:
                GPIO.output(LED_R, GPIO.LOW)
            if t < g / 255 * 50:
                GPIO.output(LED_G, GPIO.HIGH)
            else:
                GPIO.output(LED_G, GPIO.LOW)
            if t < b / 255 * 50:
                GPIO.output(LED_B, GPIO.HIGH)
            else:
                GPIO.output(LED_B, GPIO.LOW)
            time.sleep(0.03)
    except:
        print "except"
    #使用try except语句，当CTRL+C结束进程时会触发异常后
    #会执行gpio.cleanup()语句清除GPIO管脚的状态
    GPIO.cleanup()


if __name__ == '__main__':
    doing()

