#!/usr/bin/python
# coding: utf-8

"""
# TurtleBot3 ROS Topicを読み込み、サーボ/モーターを制御するクラス

# ROS MasterのROS_MASTER_URIをlocalhostにすると、外部に配信されなくなります。 

# TX2
192.168.0.48
export ROS_MASTER_URI=http://192.168.0.48:11311/
export ROS_IP=192.168.0.48
source /home/ubuntu/catkin_ws/install_isolated/setup.bash

rostopic echo /cmd_vel
"""

from lib.servo import Servo
from lib.motor import Motor
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
        self.back_in = False
        self.back_start_time = None
        return

    def __del__(self):
        self.steering.set_angle(self.STEERING_NEUTRAL, delay=0)
        self.motor.set_speed(self.MOTOR_NEUTRAL_SPEED, delay=0)
        return

    def listener(self):
        """
        READ ROS TOPICS AND RUN FUNCTION BY HZ.

        rosnode list
        rosnode info /twist_filter
        rostopic echo /cmd_vel
        """
        rospy.init_node('ROSCarNano', anonymous=True)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        return

    def cmd_vel(self, values):
        """
        ステアリング・速度のターゲット値の処理

        ステアリング値のomegaはターゲット角速度である。現在の速度を加味してomegaを算出しなければならない

        self.target_speed: target speed (m/s)
        """
        """
        Autoware 1.9.1にバック機能は無い。
        そのため速度は負の値は正の値として扱う。
        """
        print("/cmd_vel: {}".format(values))
        speed = abs(values.linear.x)
        target_speed = map(speed, 0.0, 0.22, 0.0, 30.0)

        """
        Autowareの角速度は右カーブがマイナス、左カーブがプラス。
        omega: rad/s and 10 Hz
        1 processing = 1 Hz = 0.1 * omega
        left/right = -1 * omega
        """
        omega = values.angular.z

        """
        omegaはtarget_speedが時速2kmの時、ベストマッチ。
        速度が上がった時、それに応じてomegaを減算させる。
        """
        print("omega: {} speed: {}".format(omega, speed))
        self.set_speed(target_speed)
        self.set_angle_rad(omega)

        return

    def omega_to_angle(self, omega):
        """
        角速度ωをサーボ可動域に変換する。
        omegaは右カーブがマイナス、左カーブがプラス。
        サーボは右カーブが90-180度、左カーブが0-90度となるため、
        omegaの正負を逆にしてdegreeに変換してから90度加える。
        ω(rad/s)
        θ= ω*180/pi (deg/s)
        rad = θ*pi/180
        """
        omega = -1.0 * omega
        theta = float(omega)*float(180)/math.pi
        angle = 90.0 - (180.0 - theta)/2.0
        return angle

    def set_angle_rad(self, omega):
        """
        omega: rad/s
        """
        steering_angle = self.omega_to_angle(omega)
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
        self.motor.set_speed(motor_speed)

    def brake(self, value):
        if value.data:
            self.motor.set_speed(self.MOTOR_NEUTRAL_SPEED, delay=0)
