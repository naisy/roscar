#!/usr/bin/python
# coding: utf-8

"""
# ROS Topicを読み込み、サーボ/モーターを制御するクラス

# ROS MasterのROS_MASTER_URIをlocalhostにすると、外部に配信されなくなります。 

## Car
# 受信
# ROS Master(car)
# ip_address=192.168.0.56
export ROS_MASTER_URI=http://192.168.0.56:11311/
export ROS_IP=192.168.0.56
roscore&

## Controller
# 送信
# ROS Client(controller)
# ip_address=192.168.0.xxx
export ROS_MASTER_URI=http://192.168.0.56:11311/
export ROS_IP=192.168.0.56

## Controller
# 送信コマンド
# rostopic list
# rostopic type /steer/angle_deg
# rostopic pub /steer/angle_deg std_msgs/Int8 45
# rostopic pub /motor/speed_deg std_msgs/Int8 100
# rostopic pub /motor/brake_bool std_msgs/Bool True

## Car
# 受信コマンド
# rostopic hz /steer/angle_deg
# rostopic echo /steer/angle_deg
# rostopic echo /motor/speed_deg
# rostopic echo /motor/brake_bool
"""

from lib.servo import Servo
from lib.motor import Motor
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Bool

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
        return

    def listener(self):
        """
        READ ROS TOPICS AND RUN FUNCTION BY HZ.
        angle_deg: -45 to 45
        speed_deg: -100 to 100
        brake_bool: True or False
        """
        rospy.init_node('RoboCarROS', anonymous=True)
        rospy.Subscriber('/steer/angle_deg', Int8, self.set_angle)
        rospy.Subscriber('/motor/speed_deg', Int8, self.set_speed)
        rospy.Subscriber('/motor/brake_bool', Bool, self.brake)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        return

    def set_angle(self, angle):
        """
        angle.data: -45 to 45 degree.
        """
        steering_angle = angle.data
        steering_angle = int(float(steering_angle) * float(self.STEERING_RATE))
        steering_angle = self.STEERING_NEUTRAL + steering_angle
        """
        Adjust within operable angle
        """
        if steering_angle > self.MAX_STEERING_ANGLE:
            steering_angle = self.MAX_STEERING_ANGLE
        if steering_angle < self.MIN_STEERING_ANGLE:
            steering_angle = self.MIN_STEERING_ANGLE

        self.steering.set_angle(steering_angle)

    def set_speed(self, speed):
        motor_speed = speed.data
        if motor_speed > 0:
            motor_speed = int(float(motor_speed) * float(self.MOTOR_FORWARD_SPEED_RATE))
        elif motor_speed < 0:
            motor_speed = int(float(motor_speed) * float(self.MOTOR_BACK_SPEED_RATE))
        self.motor.set_speed(motor_speed)

    def brake(self, value):
        if value.data:
            self.motor.set_speed(self.MOTOR_NEUTRAL_SPEED, delay=0)
