# -*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time

#RGB三色灯引脚定义
LED_R = 22
LED_G = 27
LED_B = 24

#超声波引脚定义
EchoPin = 0
TrigPin = 1


#设置RGB三色灯为BCM编码方式
GPIO.setmode(GPIO.BCM)

#忽略警告信息
GPIO.setwarnings(False)
#RGB三色灯设置为输出模式
GPIO.setup(LED_R, GPIO.OUT)
GPIO.setup(LED_G, GPIO.OUT)
GPIO.setup(LED_B, GPIO.OUT)

GPIO.setup(EchoPin, GPIO.IN)
GPIO.setup(TrigPin, GPIO.OUT)

#超声波函数
def distance_test():
    GPIO.output(TrigPin,GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(TrigPin,GPIO.LOW)
    while not GPIO.input(EchoPin):
        pass
    t1 = time.time()
    while GPIO.input(EchoPin):
        pass
    t2 = time.time()
    print "distance is %d " % (((t2 - t1)* 340 / 2) * 100)
    time.sleep(0.01)
    return ((t2 - t1) * 340 / 2) * 100


def light(r, g, b):
    if r:
        GPIO.output(LED_R, GPIO.HIGH)
    else:
        GPIO.output(LED_R, GPIO.LOW)
    if g:
        GPIO.output(LED_G, GPIO.HIGH)
    else:
        GPIO.output(LED_G, GPIO.LOW)
    if b:
        GPIO.output(LED_B, GPIO.HIGH)
    else:
        GPIO.output(LED_B, GPIO.LOW)

def doing():
    try:
        while True:
            distance = distance_test()
            # if distance > 50:
            #     light(False, True, False)
            # elif distance > 30:
            #     light(True, True, False)
            # else:
            #     light(True, False, False)
    except KeyboardInterrupt:
        pass
    GPIO.cleanup()


if __name__ == '__main__':
    doing()



