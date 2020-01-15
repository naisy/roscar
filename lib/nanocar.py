#!/usr/bin/python
# coding: utf-8

"""
# 1/10 RC Carモデル
# Teb Local Plannerのcar likeをベース
# アッカーマン運動力学に基づく角度、速度を配信
# サーボ/モーターを制御
"""

from lib.servo import Servo
from lib.motor import Motor
from lib.cmd_vel_to_ackermann_drive import AckermannPublisher
from ackermann_msgs.msg import AckermannDriveStamped
import rospy
from geometry_msgs.msg import Twist
import math
import time

def map(x, in_min, in_max, out_min, out_max):
    value = (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min
    if value < out_min:
        value = out_min
    elif value > out_max:
        value = out_max
    return value

class RosCar():

    def __init__(self, cfg):
        self.STEERING_NEUTRAL     = cfg['servo_neutral_angle']
        self.MIN_STEERING_ANGLE   = cfg['servo_min_angle_limit'] # Steering left/right max angle.
        self.MAX_STEERING_ANGLE   = cfg['servo_max_angle_limit'] # Steering left/right max angle.
        self.STEERING_RATE        = cfg['steering_rate'] # Server steering angle is fomula angle. But car_client depends on car. Drift type steering, normal steering, etc. Therefore, use this rate for ajust good angle.
        self.MOTOR_NEUTRAL_SPEED  = cfg['motor_neutral_speed']
        self.MOTOR_FORWARD_SPEED_RATE = float(cfg['motor_max_speed_limit'])/float(100) # Server max speed is 100. But car_client depends on config parameter.
        self.MOTOR_BACK_SPEED_RATE    = float(cfg['motor_min_speed_limit'])/float(-100) # Server min speed is -100. But car_client depends on config parameter.

        self.steering = Servo(cfg)
        self.motor = Motor(cfg)
        self.steering.set_angle(self.STEERING_NEUTRAL, delay=0)
        self.motor.set_speed(self.MOTOR_NEUTRAL_SPEED, delay=0)

        self.speed = self.MOTOR_NEUTRAL_SPEED
        return

    def __del__(self):
        self.steering.set_angle(self.STEERING_NEUTRAL, delay=0)
        self.motor.set_speed(self.MOTOR_NEUTRAL_SPEED, delay=0)
        return

    def listener(self):
        """
        ROS Topicの更新時に処理を行う設定
        rosnode list
        rostopic echo /cmd_vel
        rostopic echo /ackermann_cmd
        """
        rospy.init_node('ROSCarNano', anonymous=True)

        """
        アッカーマン運動力学
        """
        ackermann = AckermannPublisher()
        ackermann.add()
        rospy.Subscriber('/ackermann_cmd', AckermannDriveStamped, self.ackermann_cmd)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        return

    def ackermann_cmd(self, values):
        """
        ステアリング・速度のターゲット値の処理
        ステアリング値のsteering_angleは角度(rad)なので、degreeに変換する
        speed: m/s
        target_speed: motor power %
          # map(指示を受けた速度(m/s), 指示速度の後方最高速度m/s, 指示速度の最高速度m/s, モーター後方最高出力%, モーター前方最高出力%)
          # ここでは前後モーター最高出力は100%として、デバイス設定時に車両設定の速度制限を適用する。
        """
        print("/ackermann_cmd: {}".format(values))
        #speed = abs(values.drive.speed) # no backwords
        speed = values.drive.speed
        target_speed = map(speed, -0.28, 0.28, -100.0, 100.0)
        
        """
        ステアリング角
        アッカーマン運動力学のステアリング角を利用する
        """
        angle_rad = values.drive.steering_angle
        self.set_speed(target_speed)
        self.set_angle_rad(angle_rad)

    def rad_to_angle(self, angle_rad):
        """
        angle_radをサーボ可動域に変換する。
        angle_radは右カーブがマイナス、左カーブがプラス。
        サーボは右カーブが90-180度、左カーブが0-90度となるため、
        angle_radの正負を逆にしてdegreeに変換してから90度加える。
        ω(rad/s)
        θ= angle_rad*180/pi (deg/s)
        rad = θ*pi/180
        """
        angle_rad = -1.0 * angle_rad
        theta = float(angle_rad)*float(180)/math.pi
        angle = 90.0 - (180.0 - theta)/2.0
        return angle

    def set_angle_rad(self, angle_rad):
        """
        omega: rad/s
        """
        steering_angle = self.rad_to_angle(angle_rad)
        #print("omega to angle: {} -> {}".format(omega, steering_angle))
        steering_angle = int(float(steering_angle) * float(self.STEERING_RATE))
        steering_angle = self.STEERING_NEUTRAL + steering_angle
        """
        Adjust within operable angle
        """
        if steering_angle > self.MAX_STEERING_ANGLE:
            steering_angle = self.MAX_STEERING_ANGLE
        if steering_angle < self.MIN_STEERING_ANGLE:
            steering_angle = self.MIN_STEERING_ANGLE

        print("steering: {}".format(steering_angle))
        self.steering.set_angle(steering_angle)

    def set_speed(self, speed):
        """
        speed: Analog int value for PCA9685
        """
        motor_speed = speed
        if motor_speed > 0:
            motor_speed = int(float(motor_speed) * float(self.MOTOR_FORWARD_SPEED_RATE))
        elif motor_speed < 0:
            motor_speed = int(float(motor_speed) * float(self.MOTOR_BACK_SPEED_RATE))
        print("motor: {}".format(motor_speed))
        self.motor.set_speed(motor_speed)

    def brake(self, value):
        if value.data:
            self.motor.set_speed(self.MOTOR_NEUTRAL_SPEED, delay=0)

