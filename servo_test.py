# coding: utf-8
import os
import time
import sys
import yaml
from lib.servo import Servo

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

    steering = None
    try:
        """
        LOAD SETUP VARIABLES
        """
        cfg = load_config()

        steering = Servo(cfg)

        delay = 0.4
        angle = 60
        steering.set_angle(angle)
        time.sleep(1)
        for i in range(10):
            angle = 120
            steering.set_angle(angle, delay)
            time.sleep(0.5)
            angle = 90
            steering.set_angle(angle, delay)
            time.sleep(0.5)
            angle = 120
            steering.set_angle(angle)
            time.sleep(0.5)
            angle = 60
            steering.set_angle(angle, delay)
            time.sleep(0.5)
            

    except:
        import traceback
        traceback.print_exc()
    finally:
        if steering is not None:
            angle = 90
            steering.set_angle(angle)

    print("end")

if __name__ == '__main__':
    main()
