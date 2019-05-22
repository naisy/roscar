# coding: utf-8
from lib.PCA9685 import PCA9685
import time
import threading

import pkg_resources
SMBUS='smbus'
for dist in pkg_resources.working_set:
    #print(dist.project_name, dist.version)
    if dist.project_name == 'smbus':
        break
    if dist.project_name == 'smbus2':
        SMBUS='smbus2'
        break
if SMBUS == 'smbus':
    import smbus
elif SMBUS == 'smbus2':
    import smbus2 as smbus

import logging
# ログ設定
logging.basicConfig(level=logging.DEBUG,
                    format='[%(levelname)s] time:%(created).8f pid:%(process)d pn:%(processName)-10s tid:%(thread)d tn:%(threadName)-10s fn:%(funcName)-10s %(message)s',
)

class MotorSpeedError(Exception):
    pass

class SpeedWorker(threading.Thread):

    def __init__(self, group=None, target=None, name=None,
                 args=(), kwargs=None):
        threading.Thread.__init__(self, group=group, target=target, name=name)
        self.lock = threading.Lock()
        self.parent_instance = kwargs[0]
        self.target_speed = kwargs[1]
        self.default_delay = kwargs[2]
        self.delay = self.default_delay
        if self.parent_instance is not None:
            self.set_parameter(self.target_speed, self.delay)
        return

    def set_parameter(self, speed, delay=None):
        try:
            with self.lock:
                #print("set_parameter:{} {}".format(speed,delay))
                self.target_speed = speed
                self.delay = delay
                '''
                speed: 0 to 100.
                '''
                self.target_analog = self.parent_instance.speed_to_analog(self.target_speed)

                if self.delay is None:
                    delay = self.default_delay
                start_analog = self.parent_instance.get_analog()
                self.start_speed = self.parent_instance.analog_to_speed(start_analog)
                #print("motor set_speed:{}({}) -> {}({}) delay:{}".format(self.start_speed,start_analog,speed,self.target_analog,delay))
                if start_analog == self.target_analog:
                    self.step = 0
                    return
                if delay == 0:
                    self.parent_instance.set_analog(self.target_analog)
                    self.step = 0
                    return

                if self.start_speed >= self.target_speed:
                    self.step = -1
                if self.start_speed <= self.target_speed:
                    self.step = +1

                self.step_delay = float(self.delay)/float(abs(self.parent_instance.MOTOR_MAX_SPEED - self.parent_instance.MOTOR_MIN_SPEED))
        except:
            import traceback
            traceback.print_exc()
        finally:
            self.set_keep_alive(5)
        return

    def set_keep_alive(self, alive_time):
        with self.lock:
            print("set_keep_alive")
            self.keep_alive_time = alive_time
            # スレッド実行中ならend_timeを延長する
            if self.isAlive():
                self.end_time = time.time() + alive_time
        return

    def get_end_time(self):
        with self.lock:
            return self.end_time

    def run(self):
        logging.debug("---------- start speed worker ----------")
        # スレッド開始時にend_timeを設定する
        self.end_time = time.time() + self.keep_alive_time

        while self.get_end_time() > time.time():
            now_analog = self.parent_instance.get_analog()
            if now_analog != self.target_analog:
                speed = self.start_speed
                while True:
                    with self.lock:
                        # delay == 0の割り込みによる終了確認を行う
                        now_analog = self.parent_instance.get_analog()
                        now_speed = self.parent_instance.analog_to_speed(now_analog)
                        if now_analog == self.target_analog:
                            print("Motor speed interrupt break. start:{} target:{} now:{}".format(self.start_speed,speed,now_speed))
                            break
                        # 割り込み速度変更によるanalog値の変化があったかどうか確認する
                        speed_analog = self.parent_instance.speed_to_analog(speed)
                        if speed_analog != now_analog:
                            speed = now_speed
                        else:
                            speed += self.step
                        analog = self.parent_instance.speed_to_analog(speed)
                        self.parent_instance.set_analog(analog)
                        #print(analog)
                        if analog == self.target_analog:
                            now_analog = self.parent_instance.get_analog()
                            now_speed = self.parent_instance.analog_to_speed(now_analog)
                            if not analog == now_analog:
                                msg = 'Motor speed error. Couldn\'t move '+str(self.start_speed)+" to "+str(self.target_speed)+". Now "+str(now_speed)+"."
                                raise MotorSpeedError(Exception(msg))
                            #else:
                            #    print("Motor speed ok. start:{} target:{} now:{}".format(self.start_speed,speed,now_speed))
                            break
                    time.sleep(self.step_delay)
                            
            time.sleep(0.1)

class Motor():
    '''
    PWMモーター回転を制御するクラス
    motor = Motor(cfg)
    '''

    def __init__(self, cfg):
        try:
            # モーター速度
            self.MOTOR_MIN_PULSE = cfg['motor_min_pulse'] # モーターの速度が最大前進速度になるHIGH時間のμ秒。ESC毎に特性が異なる。
            self.MOTOR_MAX_PULSE = cfg['motor_max_pulse'] # モーターの速度が最大後進速度になるHIGH時間のμ秒。ESC毎に特性が異なる。
            self.MOTOR_NEUTRAL_PULSE = cfg['motor_neutral_pulse'] # モーターのニュートラル位置
            self.MOTOR_MIN_START_PULSE = cfg['motor_min_start_pulse'] # モーターの前進が開始されるHIGH時間のμ秒。ESC毎に特性が異なる。
            self.MOTOR_MAX_START_PULSE = cfg['motor_max_start_pulse'] # モーターの後進が開始されるHIGH時間のμ秒。ESC毎に特性が異なる。
            self.MOTOR_MIN_SPEED = cfg['motor_min_speed'] # モーターのソフトウェア制御上の速度設定。motor_max_pulseの時のソフトウェア速度値。
            self.MOTOR_MAX_SPEED = cfg['motor_max_speed'] # モーターのソフトウェア制御上の速度設定。motor_min_pulseの時のソフトウェア速度値。
            self.MOTOR_NEUTRAL_SPEED = cfg['motor_neutral_speed'] # 停止時の速度。
            self.MOTOR_DELAY = cfg['motor_delay'] # モーターの回転速度遅延。
            self.MOTOR_HZ = cfg['motor_hz'] # PWM周期 PCA9685設定。全てのchannelで同じ値が適用される（PCA9685仕様）。TEU-105BKは57.8Hz。
            self.MOTOR_MIN_SPEED_LIMIT = cfg['motor_min_speed_limit'] # 後進の制限速度。
            self.MOTOR_MAX_SPEED_LIMIT = cfg['motor_max_speed_limit'] # 前進の制限速度。
            self.CHANNEL = cfg['motor_channel'] # PCA9685 サーボ接続チャネル。
            self.BUSNUM = cfg['motor_busnum'] # PCA9685 I2C Bus number.
            self.I2C_ADDRESS = cfg['motor_i2c_address'] # PCA9685 I2C Address.
            self.cfg = cfg
            self.bus = smbus.SMBus(self.BUSNUM)
            init_analog = self.speed_to_analog(self.MOTOR_NEUTRAL_SPEED)
            self.PCA9685 = PCA9685(bus=self.bus, value=init_analog, address=self.I2C_ADDRESS)
            self.PCA9685.set_hz(self.MOTOR_HZ)
            print("hz:{}".format(self.MOTOR_HZ))
            self.speed_worker = None
            self.lock = threading.Lock()
        except:
            import traceback
            traceback.print_exc()
        return

    def translate(self, value, leftMin, leftMax, rightMin, rightMax):
        # Figure out how 'wide' each range is
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - leftMin) / float(leftSpan)

        # Convert the 0-1 range into a value in the right range.
        return rightMin + (valueScaled * rightSpan)

    def speed_to_analog(self, speed):
        '''
        speed: -100 to 100.
        return: PWM value.
        Resolution is different for forward and backward, so separate processing.
        '''
        if speed == 0:
            pulse = self.MOTOR_NEUTRAL_PULSE
        elif speed > 0:
            pulse = self.translate(speed, self.MOTOR_MAX_SPEED, 0, self.MOTOR_MIN_PULSE, self.MOTOR_MIN_START_PULSE)
        else:
            pulse = self.translate(speed, self.MOTOR_MIN_SPEED, 0, self.MOTOR_MAX_PULSE, self.MOTOR_MAX_START_PULSE)
        analog = round(float(pulse) / 1000000 * self.MOTOR_HZ * 4096)
        return analog

    def speed_to_pulse(self, speed):
        '''
        FOR DEBUG
        speed: -100 to 100.
        return: PWM value.
        Resolution is different for forward and backward, so separate processing.
        '''
        if speed == 0:
            pulse = self.MOTOR_NEUTRAL_PULSE
        elif speed > 0:
            pulse = self.translate(speed, self.MOTOR_MAX_SPEED, 0, self.MOTOR_MIN_PULSE, self.MOTOR_MIN_START_PULSE)
        else:
            pulse = self.translate(speed, self.MOTOR_MIN_SPEED, 0, self.MOTOR_MAX_PULSE, self.MOTOR_MAX_START_PULSE)
        return pulse

    def analog_to_speed(self, analog):
        '''
        analog: PWM value.
        return: -100 to 100.
        Resolution is different for forward and backward, so separate processing.
        '''
        pulse = float(analog) * 1000000 / self.MOTOR_HZ / 4096
        if self.MOTOR_MIN_START_PULSE <= pulse and pulse <= self.MOTOR_MAX_START_PULSE:
            speed = 0
        elif pulse < self.MOTOR_MIN_START_PULSE:
            speed = round(self.translate(pulse, self.MOTOR_MIN_PULSE, self.MOTOR_MIN_START_PULSE, self.MOTOR_MAX_SPEED, 0))
        else:
            speed = round(self.translate(pulse, self.MOTOR_MAX_PULSE, self.MOTOR_MAX_START_PULSE, self.MOTOR_MIN_SPEED, 0))
        return speed

    def analog_to_pulse(self, analog):
        '''
        FOR DEBUG
        '''
        speed = self.analog_to_speed(analog)
        pulse = self.speed_to_pulse(speed)
        return pulse
        
    def get_analog(self):
        return self.PCA9685.get_channel_value(self.CHANNEL)

    def set_analog(self, analog):
        self.PCA9685.set_channel_value(self.CHANNEL, analog)

    def set_speed(self, speed, delay=None):
        with self.lock:
            if delay is None:
                delay = self.MOTOR_DELAY
            if self.speed_worker is None or not self.speed_worker.isAlive():
                logging.debug("t create")
                self.speed_worker = SpeedWorker(kwargs=[self, speed, delay])
                self.speed_worker.start()
            else:
                logging.debug("t alive")
                self.speed_worker.set_parameter(speed=speed, delay=delay)
        return

    def get_speed(self):
        '''
        return: -100 to 100.
        '''
        try:
            analog = self.get_analog()
            return self.analog_to_speed(analog)
        except:
            import traceback
            traceback.print_exc()

    def neutral(self, value=None):
        '''
        value: -100 to 100.
        ちょうどいい初期位置が中央位置とも限らないので、ここはニュートラル位置と呼ぶことにする
        '''
        try:
            if value is None:
                value = self.MOTOR_NEUTRAL_SPEED
            '''
            サーボをニュートラル位置に戻す
            引数valueはニュートラル位置を更新する
            '''
            if not self.motor_speed_validation(value):
                return

            # 引数valueをニュートラル位置に更新する
            self.MOTOR_NEUTRAL_SPEED = value
            # モーターをニュートラル位置に設定する
            self.set_speed(self.MOTOR_NEUTRAL_SPEED)
        except:
            import traceback
            traceback.print_exc()

    def motor_speed_validation(self,value):
        '''
        value: -100 to 100.
        '''
        try:
            '''
            引数valueがモーターの制限速度以内かどうかを確認する
            '''
            # バリデーション: MOTOR_MIN_SPEED_LIMIT <= value <= MOTOR_MAX_SPEED_LIMIT
            if not (self.MOTOR_MIN_SPEED_LIMIT <= value):
                return False
            if not (value <= self.MOTOR_MAX_SPEED_LIMIT):
                return False
            return True
        except:
            import traceback
            traceback.print_exc()
