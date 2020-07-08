# -*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time
from threading import Thread

class Const:

    def __init__(self):
        # 超声波引脚定义
        self.EchoPin = 0
        self.TrigPin = 1

        # 小车电机引脚定义
        self.IN1 = 20
        self.IN2 = 21
        self.IN3 = 19
        self.IN4 = 26
        self.ENA = 16
        self.ENB = 13

        # RGB三色灯引脚定义
        self.LED_R = 22
        self.LED_G = 27
        self.LED_B = 24

        # 小车按键定义
        self.KEY = 8

        # 循迹红外引脚定义
        # TrackSensorLeftPin1 TrackSensorLeft/Pin2 TrackSensorRightPin1 TrackSensorRightPin2
        #      3                 5                  4                   18
        self.TrackSensorLeftPin1 = 3  # 定义左边第一个循迹红外传感器引脚为3口
        self.TrackSensorLeftPin2 = 5  # 定义左边第二个循迹红外传感器引脚为5口
        self.TrackSensorRightPin1 = 4  # 定义右边第一个循迹红外传感器引脚为4口
        self.TrackSensorRightPin2 = 18  # 定义右边第二个循迹红外传感器引脚为18口

        self.SPEED_FAST = 30  # 小车的快速
        self.SPEED_MIDDLE = 25  # 小车的中速
        self.SPEED_SLOW = 20  # 小车的慢速
        self.SPEED_VERY_SLOW = 20
        self.RUN_SLEEP_CATCH = 0.005
        self.LIGHT_SLEEP = 0.01
        self.TRACK_SENSOR_SLEEP = 0.01
        self.SONIC_SLEEP = 0.01
        self.SONIC_SLEEP_SPACE = 10
        self.SONIC_DISTANCE = 10
        self.DURING = 2


class ShareState:
    def __init__(self):
        self.STOP = False
        self.car_state = 1
        self.track_sensor = [1, 1, 1, 1]
        self.track_sensor_old = [1, 1, 1, 1]
        self.distance = 100
        self.distance_old = 100
        self.EVENT_TRACK_SENSOR = None
        self.EVENT_SONIC = None
        self.THREAD_LIGHT = LightThread()
        self.TRACK_SENSOR_THREAD = TrackSensorThread()
        self.SONIC_THREAD = SonicThread()
        self.CAR_THREAD = CarThread()
        self.time_old = 0
        self.ENABLE_SONIC = 1
        self.R = 0
        self.G = 0
        self.B = 0

    def default_event_track_sensor(self):
        print("track_sensor changed:" + ",".join(map(str, shareState.track_sensor)))
        print("carstate" + str(self.car_state))

    def default_event_sonic(self):
        print("distance: " + str(shareState.distance))

    def set_time(self, time):
        if time - self.time_old > const.SONIC_SLEEP_SPACE:
            self.ENABLE_SONIC = 1
            self.time_old = time
        else:
            self.ENABLE_SONIC = 0


    def is_in_track(self, track_sensor):
        if track_sensor[1] == 0 or track_sensor[2] == 0:
            return True
        else:
            return False

    def set_color(self, r, g, b):
        self.R = r
        self.G = g
        self.B = b

    def set_track_sensor(self, track_sensor):
        if track_sensor != self.track_sensor:
            self.track_sensor_old = self.track_sensor
            self.track_sensor = track_sensor
            if self.is_in_track(self.track_sensor):
                self.car_state = 0
            elif self.is_in_track(self.track_sensor_old):
                self.car_state = 2
            else:
                self.car_state = 1

            if self.EVENT_TRACK_SENSOR is not None:
                self.EVENT_TRACK_SENSOR()
            else:
                self.default_event_track_sensor()

    def set_distance(self, distance):
        self.distance_old = self.distance
        self.distance = distance
        if self.EVENT_SONIC is not None:
            self.EVENT_SONIC()
        else:
            self.default_event_sonic()


class LightThread(Thread):
    def __init__(self):
        # RGB三色灯设置为输出模式
        super(LightThread, self).__init__()
        # 设置GPIO口为BCM编码方式
        GPIO.setmode(GPIO.BCM)
        # 忽略警告信息
        GPIO.setwarnings(False)
        GPIO.setup(const.LED_R, GPIO.OUT)
        GPIO.setup(const.LED_G, GPIO.OUT)
        GPIO.setup(const.LED_B, GPIO.OUT)

    def run(self):
        while not shareState.STOP:
            self.__init__()
            try:
                if shareState.R == 0:
                    GPIO.output(const.LED_R, GPIO.LOW)
                else:
                    GPIO.output(const.LED_R, GPIO.HIGH)
                if shareState.G == 0:
                    GPIO.output(const.LED_G, GPIO.LOW)
                else:
                    GPIO.output(const.LED_G, GPIO.HIGH)
                if shareState.B == 0:
                    GPIO.output(const.LED_B, GPIO.LOW)
                else:
                    GPIO.output(const.LED_B, GPIO.HIGH)
                time.sleep(const.LIGHT_SLEEP)
            except RuntimeError:
                pass


class TrackSensorThread(Thread):
    def __init__(self):
        super(TrackSensorThread, self).__init__()
        GPIO.setmode(GPIO.BCM)
        # 忽略警告信息
        GPIO.setwarnings(False)
        GPIO.setup(const.TrackSensorLeftPin1, GPIO.IN)
        GPIO.setup(const.TrackSensorLeftPin2, GPIO.IN)
        GPIO.setup(const.TrackSensorRightPin1, GPIO.IN)
        GPIO.setup(const.TrackSensorRightPin2, GPIO.IN)

    def track_sensor(self):
        return [
            1 - GPIO.input(const.TrackSensorLeftPin1),
            1 - GPIO.input(const.TrackSensorLeftPin2),
            1 - GPIO.input(const.TrackSensorRightPin1),
            1 - GPIO.input(const.TrackSensorRightPin2),
        ]

    def run(self):
        while not shareState.STOP:
            self.__init__()
            try:
                time.sleep(const.TRACK_SENSOR_SLEEP)
                shareState.set_track_sensor(self.track_sensor())
            except RuntimeError:
                pass


class SonicThread(Thread):
    def __init__(self):
        super(SonicThread, self).__init__()
        GPIO.setmode(GPIO.BCM)
        # 忽略警告信息
        GPIO.setwarnings(False)
        GPIO.setup(const.EchoPin, GPIO.IN)
        GPIO.setup(const.TrigPin, GPIO.OUT)

    @staticmethod
    def get_distance():
        GPIO.output(const.TrigPin, GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(const.TrigPin, GPIO.LOW)
        while not GPIO.input(const.EchoPin):
            pass
        t1 = time.time()
        while GPIO.input(const.EchoPin):
            pass
        t2 = time.time()
        time.sleep(const.TRACK_SENSOR_SLEEP)
        value = ((t2 - t1) * 340 / 2) * 100
        return value

    def run(self):
        while not shareState.STOP:
            try:
                self.__init__()
                distances = []
                for i in range(0, 3):
                    distances.append(self.get_distance())
                value = min(distances)
                shareState.set_distance(value)
            except RuntimeError:
                pass


class CarThread(Thread):
    def __init__(self):
        global pwm_ENA
        global pwm_ENB
        # 设置GPIO口为BCM编码方式
        super(CarThread, self).__init__()
        GPIO.setmode(GPIO.BCM)
        # 忽略警告信息
        GPIO.setwarnings(False)
        GPIO.setup(const.ENA, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(const.IN1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(const.IN2, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(const.ENB, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(const.IN3, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(const.IN4, GPIO.OUT, initial=GPIO.LOW)
        # 设置pwm引脚和频率为2000hz
        pwm_ENA = GPIO.PWM(const.ENA, 2000)
        pwm_ENB = GPIO.PWM(const.ENB, 2000)
        pwm_ENA.start(0)
        pwm_ENB.start(0)

    # 小车前进
    def _run(self, leftspeed, rightspeed):
        GPIO.output(const.IN1, GPIO.HIGH)
        GPIO.output(const.IN2, GPIO.LOW)
        GPIO.output(const.IN3, GPIO.HIGH)
        GPIO.output(const.IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(leftspeed)
        pwm_ENB.ChangeDutyCycle(rightspeed)

    # 小车后退
    def _back(self, leftspeed, rightspeed):
        GPIO.output(const.IN1, GPIO.LOW)
        GPIO.output(const.IN2, GPIO.HIGH)
        GPIO.output(const.IN3, GPIO.LOW)
        GPIO.output(const.IN4, GPIO.HIGH)
        pwm_ENA.ChangeDutyCycle(leftspeed)
        pwm_ENB.ChangeDutyCycle(rightspeed)

    # 小车左转
    def _left(self, leftspeed, rightspeed):
        GPIO.output(const.IN1, GPIO.LOW)
        GPIO.output(const.IN2, GPIO.LOW)
        GPIO.output(const.IN3, GPIO.HIGH)
        GPIO.output(const.IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(leftspeed)
        pwm_ENB.ChangeDutyCycle(rightspeed)

    # 小车右转
    def _right(self, leftspeed, rightspeed):
        GPIO.output(const.IN1, GPIO.HIGH)
        GPIO.output(const.IN2, GPIO.LOW)
        GPIO.output(const.IN3, GPIO.LOW)
        GPIO.output(const.IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(leftspeed)
        pwm_ENB.ChangeDutyCycle(rightspeed)

    # 小车原地左转
    def _spin_left(self, leftspeed, rightspeed):
        GPIO.output(const.IN1, GPIO.LOW)
        GPIO.output(const.IN2, GPIO.HIGH)
        GPIO.output(const.IN3, GPIO.HIGH)
        GPIO.output(const.IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(leftspeed)
        pwm_ENB.ChangeDutyCycle(rightspeed)

    # 小车原地右转
    def _spin_right(self, leftspeed, rightspeed):
        GPIO.output(const.IN1, GPIO.HIGH)
        GPIO.output(const.IN2, GPIO.LOW)
        GPIO.output(const.IN3, GPIO.LOW)
        GPIO.output(const.IN4, GPIO.HIGH)
        pwm_ENA.ChangeDutyCycle(leftspeed)
        pwm_ENB.ChangeDutyCycle(rightspeed)

    # 小车停止
    def _brake(self):
        GPIO.output(const.IN1, GPIO.LOW)
        GPIO.output(const.IN2, GPIO.LOW)
        GPIO.output(const.IN3, GPIO.LOW)
        GPIO.output(const.IN4, GPIO.LOW)

    def run(self):
        while not shareState.STOP:
            try:
                if shareState.distance < const.SONIC_DISTANCE:
                    shareState.set_time(time.time())
                    if shareState.ENABLE_SONIC:
                        self._brake()
                        start = time.time()
                        shareState.set_color(1, 0, 0)
                        while True:
                            end = time.time()
                            if end - start > const.DURING:
                                break
                        shareState.set_color(0, 0, 0)




                # X 0 1 X 左小弯
                if shareState.track_sensor[1] == 0 and shareState.track_sensor[2] == 1:
                    self._left(0, const.SPEED_MIDDLE)
                # X 1 0 X 右小弯
                elif shareState.track_sensor[1] == 1 and shareState.track_sensor[2] == 0:
                    self._right(const.SPEED_MIDDLE, 0)
                # X 0 0 X 直行
                elif shareState.track_sensor[1] == 0 and shareState.track_sensor[2] == 0:
                    self._run(const.SPEED_FAST, const.SPEED_FAST)

                # X X X 0 (old) 右直转
                if shareState.track_sensor_old[3] == 0:
                    self._spin_right(const.SPEED_SLOW, const.SPEED_VERY_SLOW)

                # 0 X X X (old) 左直转
                # 1 X X 1 (old) 倒车
                else:
                    self._spin_left(const.SPEED_VERY_SLOW, const.SPEED_SLOW)
                # else 其他情况不变
            except RuntimeError:
                pass
        pwm_ENA.stop()
        pwm_ENB.stop()


class Environ:

    def __init__(self):
        # 设置GPIO口为BCM编码方式
        GPIO.setmode(GPIO.BCM)
        # 忽略警告信息
        GPIO.setwarnings(False)
        GPIO.setup(const.KEY, GPIO.IN)

    def _key_scan(self):
        while GPIO.input(const.KEY):
            pass
        while not GPIO.input(const.KEY):
            time.sleep(0.01)
            if not GPIO.input(const.KEY):
                time.sleep(0.01)
                while not GPIO.input(const.KEY):
                    pass

    def task(self):
        shareState.THREAD_LIGHT.start()
        shareState.TRACK_SENSOR_THREAD.start()
        shareState.SONIC_THREAD.start()
        shareState.CAR_THREAD.run()

    def stop(self):
        shareState.STOP = True

    def run(self):
        self._key_scan()
        try:
            self.task()
            while True:
                print("doing main thread task")
                time.sleep(10)
        except KeyboardInterrupt:
            pass
        self.stop()
        GPIO.cleanup()


const = Const()
shareState = ShareState()

if __name__ == '__main__':
    environ = Environ()
    environ.run()

