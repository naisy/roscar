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
        self.STEERING_REVERSE = cfg['servo_reverse']
        self.MIN_STEERING_ANGLE_LIMIT   = cfg['servo_min_angle_limit'] # Steering left/right max angle.
        self.MAX_STEERING_ANGLE_LIMIT   = cfg['servo_max_angle_limit'] # Steering left/right max angle.
        self.STEERING_RATE        = cfg['steering_rate'] # Server steering angle is fomula angle. But car_client depends on car. Drift type steering, normal steering, etc. Therefore, use this rate for ajust good angle.
        self.MOTOR_NEUTRAL_SPEED  = cfg['motor_neutral_speed']
        self.MOTOR_REVERSE = cfg['motor_reverse']
        self.MIN_MOTOR_SPEED_LIMIT = cfg['motor_min_speed_limit']
        self.MAX_MOTOR_SPEED_LIMIT = cfg['motor_max_speed_limit']

        self.steering = Servo(cfg)
        self.motor = Motor(cfg)
        self.steering.set_angle(self.STEERING_NEUTRAL, delay=0)
        self.motor.set_speed(self.MOTOR_NEUTRAL_SPEED, delay=0)
        return

    def set_angle(self, angle):
        """
        angle: -45 to 45 degree.
        """
        if self.STEERING_REVERSE:
            angle = -1.0 * angle
        steering_angle = angle
        steering_angle = int(float(steering_angle) * float(self.STEERING_RATE))
        steering_angle = self.STEERING_NEUTRAL + steering_angle
        """
        Adjust within operable angle
        """
        if steering_angle > self.MAX_STEERING_ANGLE_LIMIT:
            steering_angle = self.MAX_STEERING_ANGLE_LIMIT
        if steering_angle < self.MIN_STEERING_ANGLE_LIMIT:
            steering_angle = self.MIN_STEERING_ANGLE_LIMIT

        self.steering.set_angle(steering_angle)

    def set_speed(self, speed):
        if self.MOTOR_REVERSE:
            speed = -1.0 * speed
        motor_speed = speed
        if motor_speed > self.MAX_MOTOR_SPEED_LIMIT:
            motor_speed = self.MAX_MOTOR_SPEED_LIMIT
        if motor_speed < self.MIN_MOTOR_SPEED_LIMIT:
            motor_speed = self.MIN_MOTOR_SPEED_LIMIT
        self.motor.set_speed(motor_speed)

    def brake(self, value):
        if value.data:
            self.motor.set_speed(self.MOTOR_NEUTRAL_SPEED, delay=0)

