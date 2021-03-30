# coding: utf-8
import os
import time
import sys
import yaml
from lib.motor import Motor

def load_config():
    """
    LOAD CONFIG FILE
    Convert config.yml to DICT.
    """
    cfg = None
    if (os.path.isfile('config.yml')):
        with open("config.yml", 'r') as ymlfile:
            cfg = yaml.load(ymlfile)
    else:
        raise FileNotFoundError(("File not found: config.yml"))
    return cfg

def main():
    print("start")

    motor = None
    try:
        """
        LOAD SETUP VARIABLES
        """
        cfg = load_config()

        motor = Motor(cfg)
        """
        # NEUTRAL ANALOG RANGE: 370-393
        neutral = 370
        print(motor.analog_to_pulse(neutral))
        neutral = 393
        print(motor.analog_to_pulse(neutral))
        # FORWARD MAXIMUM ANALOG VALUE: 280
        analog = 280
        print(motor.analog_to_pulse(analog))
        # BACKWARD MAXIMUM ANALOG VALUE: 440
        analog = 440
        print(motor.analog_to_pulse(analog))
        """

        # FORWARD: 369(START) - 280(MAX SPEED)
        motor.set_speed(0)
        time.sleep(2)
        speed = 0
        """
        for i in range(100):
            speed += 1
            motor.set_speed(speed)
            print(motor.get_speed())
            time.sleep(0.1)
        for i in range(100):
            speed -= 1
            motor.set_speed(speed)
            print(motor.get_speed())
            time.sleep(0.1)
        """

        # Setting the "BACKWARD" value to the value will apply the brake. (It is probably the function of ESC.)

        # BACKWORD: 394(START) - 440(MAX SPEED)
        speed = -30 # 30% backword throttle
        delay = 5.0
        for i in range(2):
            speed = 30 # 30% throttle
            motor.set_speed(speed, delay)
            time.sleep(2.5)
            speed = 0
            motor.set_speed(speed, delay)
            time.sleep(2.5)
            motor.set_speed(0)
            time.sleep(2)

    except:
        import traceback
        traceback.print_exc()
    finally:
        if motor is not None:
            motor.set_speed(0)
            time.sleep(3)

    print("end")

if __name__ == '__main__':
    main()
