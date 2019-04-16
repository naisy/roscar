#!/usr/bin/python
# coding: utf-8

"""
# Autoware ROS Topicを読み込み、サーボ/モーターを制御するクラス

# ROS MasterのROS_MASTER_URIをlocalhostにすると、外部に配信されなくなります。 

[ROS Master ip_address=192.168.0.56]
export ROS_MASTER_URI=http://192.168.0.56:11311/
export ROS_IP=192.168.0.56
roscore&

[ROS Client ip_address=192.168.0.xxx]
export ROS_MASTER_URI=http://192.168.0.56:11311/
export ROS_IP=192.168.0.56
rostopic echo /twist_cmd

# Autoware 受信コマンド
# rostopic echo /twist_cmd

# Autoware /twist_cmdは geometory_msgs/TwistStamped型
# YAML構造なので、key-value型
# hydroの方言、geometry_msgs/Vector3
# 速度v(km/h)はlinear xの値。v=x
# 角速度ω(rad/s)はangularのzの値。θ=z
# 1rad = 180/pi = 57.3 deg
# ω(rad/s) = ω*180/pi (deg/s)
# 角度はθ=ω*180/pi (deg/s)

# /twist_cmdを直接読み取らないと値を取得出来ないことがある
"""

from lib.servo import Servo
from lib.motor import Motor
import rospy
#from std_msgs.msg import Float64
#from geometry_msgs.msg import Vector3
from geometry_msgs.msg import TwistStamped
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
        self.target_speed = self.MOTOR_NEUTRAL_SPEED
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
        rostopic echo /twist_cmd
        """
        rospy.init_node('RoboCarROS', anonymous=True)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd)
        # current_velocity
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        return

    def current_velocity(self, values):
        """
        現在速度と目標速度を比較して、モーター出力を調整する。
        self.speed: モーター出力
        current_velocity: 現在速度
        self.target_speed: 目標速度
        """
        current_velocity = values.twist.linear.x
        print("speed now: {} target: {}".format(current_velocity, self.target_speed))
        if current_velocity < self.target_speed:
            self.speed += 1
        elif current_velocity > self.target_speed:
            self.speed -= 1
        if self.speed >= 100:
            self.speed = 100
        elif self.speed <= 0:
            self.speed = 0
        """
        現在速度が0.1(m/s)未満の時、急発進を抑えるためにモーター出力を制限する。
        """
        if self.speed > 25 and current_velocity < 0.1:
            self.speed = 25
        self.set_speed(self.speed)
        return

    def twist_cmd(self, values):
        """
        ステアリング・速度のターゲット値の処理

        ステアリング値はここで適用する
        速度は変数で保持し、current_velocityによる現在速度取得時に適用する
        ステアリング値のomegaはターゲット角速度である。現在の速度を加味してomegaを算出しなければならない

        self.target_speed: target speed (m/s)
        """
        #print(values)
        """
        Autoware 1.9.1にバック機能は無い。
        そのため速度は負の値は正の値として扱う。
        """
        speed = abs(values.twist.linear.x)
        self.target_speed = speed

        """
        Autowareの角速度は右カーブがマイナス、左カーブがプラス。
        omega: rad/s and 10 Hz
        1 processing = 1 Hz = 0.1 * omega
        left/right = -1 * omega
        """
        omega = values.twist.angular.z

        """
        omegaはtarget_speedが時速2kmの時、ベストマッチ。
        速度が上がった時、それに応じてomegaを減算させる。
        """
        print("before omega: {} speed: {}".format(omega, speed))
        if self.target_speed >= 0.56:
            """
            目標速度が時速2km(0.56m/s)以上の時、比率に応じてomegaを小さくする
            """
            ratio = map(self.target_speed, 2.0, 1000.0, 1.0, 1000.0)
            omega = omega*ratio

        print("after omega: {} speed: {}".format(omega, speed))
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
