# -*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
from final import *
import time

speed_fast = 100
speed_middle = 90
speed_slow = 80
speed_veryslow = 40
time_sleep = 0.08


def doing():
    # 延时2s
    time.sleep(2)

    # try/except语句用来检测try语句块中的错误，
    # 从而让except语句捕获异常信息并处理。
    try:
        init()
        key_scan()
        while True:
            # 检测到黑线时循迹模块相应的指示灯亮，端口电平为LOW
            # 未检测到黑线时循迹模块相应的指示灯灭，端口电平为HIGH
            TrackSensorLeftValue1 = GPIO.input(TrackSensorLeftPin1)
            TrackSensorLeftValue2 = GPIO.input(TrackSensorLeftPin2)
            TrackSensorRightValue1 = GPIO.input(TrackSensorRightPin1)
            TrackSensorRightValue2 = GPIO.input(TrackSensorRightPin2)

            # 四路循迹引脚电平状态
            # X 0 1 X
            # 处理左小弯
            if TrackSensorLeftValue2 == False and TrackSensorRightValue1 == True:
                run(speed_veryslow, speed_middle)

            # 四路循迹引脚电平状态
            # X 1 0 X
            # 处理右小弯
            elif TrackSensorLeftValue2 == True and TrackSensorRightValue1 == False:
                run(speed_middle, speed_veryslow)

            # 四路循迹引脚电平状态
            # X 0 0 X
            # 处理直线
            elif TrackSensorLeftValue2 == False and TrackSensorRightValue1 == False:
                run(speed_fast, speed_fast)


            # 四路循迹引脚电平状态
            # 0 0 X 0
            # 1 0 X 0
            # 0 1 X 0
            # 以上6种电平状态时小车原地右转
            # 处理右锐角和右直角的转动
            elif (TrackSensorLeftValue1 == False or TrackSensorLeftValue2 == False) and TrackSensorRightValue2 == False:
                spin_right(speed_fast, speed_slow)
                time.sleep(time_sleep)

            # 四路循迹引脚电平状态
            # 0 X 0 0
            # 0 X 0 1
            # 0 X 1 0
            # 处理左锐角和左直角的转动
            elif TrackSensorLeftValue1 == False and (
                    TrackSensorRightValue1 == False or TrackSensorRightValue2 == False):
                spin_left(speed_fast, speed_slow)
                time.sleep(time_sleep)

            # 0 X X X
            # 最左边检测到
            elif TrackSensorLeftValue1 == False:
                spin_left(speed_slow, speed_slow)

            # X X X 0
            # 最右边检测到
            elif TrackSensorRightValue2 == False:
                spin_right(speed_slow, speed_slow)

            # 当为1 1 1 1时小车保持上一个小车运行状态

    except KeyboardInterrupt:
        pass
    pwm_ENA.stop()
    pwm_ENB.stop()
    GPIO.cleanup()


if __name__ == '__main__':
    doing()
