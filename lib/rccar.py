#!/usr/bin/python
# coding: utf-8

"""
# RCCar 制御クラス
"""

from lib.servo import Servo
from lib.motor import Motor

class RCCar():

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

    def set_angle(self, angle):
        """
        angle: -45 to 45 degree.
        """
        steering_angle = angle
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
        motor_speed = speed
        if motor_speed > 0:
            motor_speed = int(float(motor_speed) * float(self.MOTOR_FORWARD_SPEED_RATE))
        elif motor_speed < 0:
            motor_speed = int(float(motor_speed) * float(self.MOTOR_BACK_SPEED_RATE))
        self.motor.set_speed(motor_speed)

    def brake(self, value):
        if value.data:
            self.motor.set_speed(self.MOTOR_NEUTRAL_SPEED, delay=0)

