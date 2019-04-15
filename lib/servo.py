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

class ServoAngleError(Exception):
    pass

class AngleWorker(threading.Thread):

    def __init__(self, group=None, target=None, name=None,
                 args=(), kwargs=None):
        threading.Thread.__init__(self, group=group, target=target, name=name)
        self.lock = threading.Lock()
        self.parent_instance = kwargs[0]
        self.target_angle = kwargs[1]
        self.default_delay = kwargs[2]
        self.delay = self.default_delay
        if self.parent_instance is not None:
            self.set_parameter(self.target_angle, self.delay)
        return

    def set_parameter(self, angle, delay=None):
        try:
            with self.lock:
                self.target_angle = angle
                self.delay = delay
                '''
                angle: 0 to 180 degree.
                '''
                self.target_analog = self.parent_instance.angle_to_analog(self.target_angle)

                if self.delay is None:
                    delay = self.default_delay
                start_analog = self.parent_instance.get_analog()
                self.start_angle = self.parent_instance.analog_to_angle(start_analog)
                #print("servo set_angle:{}({}) -> {}({}) delay:{}".format(self.start_angle,start_analog,angle,self.target_analog,delay))
                if start_analog == self.target_analog:
                    self.step = 0
                    return
                if delay == 0:
                    self.parent_instance.set_analog(self.target_analog)
                    self.step = 0
                    return

                if self.start_angle >= self.target_angle:
                    self.step = -1
                if self.start_angle <= self.target_angle:
                    self.step = +1

                self.step_delay = float(self.delay)/float(abs(self.parent_instance.SERVO_MAX_ANGLE - self.parent_instance.SERVO_MIN_ANGLE))
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
        logging.debug("---------- start angle worker ----------")
        # スレッド開始時にend_timeを設定する
        self.end_time = time.time() + self.keep_alive_time

        while self.get_end_time() > time.time():
            now_analog = self.parent_instance.get_analog()
            if now_analog != self.target_analog:
                angle = self.start_angle
                while True:
                    with self.lock:
                        # delay == 0の割り込みによる終了確認を行う
                        now_analog = self.parent_instance.get_analog()
                        now_angle = self.parent_instance.analog_to_angle(now_analog)
                        if now_analog == self.target_analog:
                            print("Servo angle interrupt break. start:{} target:{} now:{}".format(self.start_angle,angle,now_angle))
                            break
                        # 割り込み速度変更によるanalog値の変化があったかどうか確認する
                        angle_analog = self.parent_instance.angle_to_analog(angle)
                        if angle_analog != now_analog:
                            angle = now_angle
                        else:
                            angle += self.step
                        analog = self.parent_instance.angle_to_analog(angle)
                        self.parent_instance.set_analog(analog)
                        #print(analog)
                        if analog == self.target_analog:
                            now_analog = self.parent_instance.get_analog()
                            now_angle = self.parent_instance.analog_to_angle(now_analog)
                            if not analog == now_analog:
                                msg = 'Servo angle error. Couldn\'t move '+str(self.start_angle)+" to "+str(self.target_angle)+". Now "+str(now_angle)+"."
                                raise ServoAngleError(Exception(msg))
                            #else:
                            #    print("Servo angle ok. start:{} target:{} now:{}".format(self.start_angle,angle,now_angle))
                            break
                    time.sleep(self.step_delay)

            time.sleep(0.1)

class Servo():
    '''
    SERVOサーボ回転を制御するクラス
    servo = Servo(cfg)
    '''

    def __init__(self, cfg):
        try:
            # サーボの限界軸角度
            self.SERVO_MIN_PULSE = cfg['servo_min_pulse'] # サーボの軸角度が0度になるHIGH時間のμ秒。サーボ毎に特性が異なる。
            self.SERVO_MAX_PULSE = cfg['servo_max_pulse'] # サーボの軸角度が180度になるHIGH時間のμ秒。サーボ毎に特性が異なる。
            self.SERVO_MIN_ANGLE = cfg['servo_min_angle'] # サーボのソフトウェア制御上の速度設定。servo_min_pulseの時のソフトウェア角度。
            self.SERVO_MAX_ANGLE = cfg['servo_max_angle'] # サーボのソフトウェア制御上の速度設定。servo_max_pulseの時のソフトウェア角度。
            self.SERVO_NEUTRAL_ANGLE = cfg['servo_neutral_angle'] # サーボのニュートラル角度。
            self.SERVO_DELAY = cfg['servo_delay'] # サーボの角速度遅延
            self.SERVO_HZ = cfg['servo_hz'] # PWM周期 PCA9685設定。全てのchannelで同じ値が適用される（PCA9685仕様）。60Hzだと壊れるサーボを使うため、50Hzに設定する。
            # サーボの限界軸角度。サーボ自体の軸角度の他に、サーボを取り付けた部分の稼働可能角を考慮して、稼働可能な軸角度を決めること
            self.SERVO_MIN_ANGLE_LIMIT = cfg['servo_min_angle_limit']
            self.SERVO_MAX_ANGLE_LIMIT = cfg['servo_max_angle_limit']
            self.CHANNEL = cfg['servo_channel'] # PCA9685 サーボ接続チャネル
            self.BUSNUM = cfg['servo_busnum'] # I2C Bus number

            self.bus = smbus.SMBus(self.BUSNUM)
            init_analog = self.angle_to_analog(self.SERVO_NEUTRAL_ANGLE)
            self.PCA9685 = PCA9685(self.bus, init_analog)
            self.PCA9685.set_hz(self.SERVO_HZ)
            print("hz:{}".format(self.SERVO_HZ))
            self.angle_worker = None
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

    def angle_to_analog(self, angle):
        '''
        angle: 0 to 180 degree.
        return: PWM value.
        '''
        pulse = self.translate(angle, self.SERVO_MIN_ANGLE, self.SERVO_MAX_ANGLE, self.SERVO_MIN_PULSE, self.SERVO_MAX_PULSE)
        analog = round(float(pulse) / 1000000 * self.SERVO_HZ * 4096)
        return analog

    def angle_to_pulse(self, angle):
        '''
        FOR DEBUG
        angle: 0 to 118 degree.
        return: PWM value.
        '''
        pulse = self.translate(angle, self.SERVO_MIN_ANGLE, self.SERVO_MAX_ANGLE, self.SERVO_MIN_PULSE, self.SERVO_MAX_PULSE)
        return pulse

    def analog_to_pulse(self, analog):
        '''
        FOR DEBUG
        '''
        angle = self.analog_to_angle(analog)
        pulse = self.angle_to_pulse(angle)
        return pulse

    def analog_to_angle(self, analog):
        '''
        analog: PWM value.
        return: 0 to 180 degree.
        '''
        pulse = float(analog) * 1000000 / self.SERVO_HZ / 4096
        angle = round(self.translate(pulse, self.SERVO_MIN_PULSE, self.SERVO_MAX_PULSE, self.SERVO_MIN_ANGLE, self.SERVO_MAX_ANGLE))
        return angle

    def get_analog(self):
        return self.PCA9685.get_channel_value(self.CHANNEL)

    def set_analog(self, analog):
        self.PCA9685.set_channel_value(self.CHANNEL, analog)

    def set_angle(self, angle, delay=None):
        '''
        angle: Value of software range.
        '''
        with self.lock:
            if delay is None:
                delay = self.SERVO_DELAY
            if self.angle_worker is None or not self.angle_worker.isAlive():
                logging.debug("t create")
                self.angle_worker = AngleWorker(kwargs=[self, angle, delay])
                self.angle_worker.start()
            else:
                logging.debug("t alive")
                self.angle_worker.set_parameter(angle=angle, delay=delay)
        return

    def get_angle(self):
        '''
        return: 0 to 180 degree.
        '''
        try:
            analog = self.get_analog()
            return self.analog_to_angle(analog)
        except:
            import traceback
            traceback.print_exc()

    def neutral(self, value=None):
        '''
        value: 0 to 180 degree.
        ちょうどいい初期位置が中央位置とも限らないので、ここはニュートラル位置と呼ぶことにする
        '''
        try:
            if value is None:
                value = self.SERVO_NEUTRAL_ANGLE
            '''
            サーボをニュートラル位置に戻す
            引数valueはニュートラル位置を更新する
            '''
            if not self.servo_angle_validation(value):
                return

            # 引数valueをニュートラル位置に更新する
            self.SERVO_NEUTRAL_ANGLE = value
            # サーボをニュートラル位置に移動する
            self.set_angle(self.SERVO_NEUTRAL_ANGLE)
        except:
            import traceback
            traceback.print_exc()

    def servo_angle_validation(self,value):
        '''
        value: 0 to 180 degree.
        '''
        try:
            '''
            引数valueがサーボの可動範囲内かどうかを確認する
            '''
            # バリデーション: SERVO_MIN_ANGLE_LIMIT <= value <= SERVO_MAX_ANGLE_LIMIT
            if not (self.SERVO_MIN_ANGLE_LIMIT <= value):
                return False
            if not (value <= self.SERVO_MAX_ANGLE_LIMIT):
                return False
            return True
        except:
            import traceback
            traceback.print_exc()


