"""
# 1/10 RC Carモデル

# ROS MasterのROS_MASTER_URIをlocalhostにすると、外部に配信されなくなります。 

# Jetson Nano (ip_address: 192.168.0.1)
export ROS_MASTER_URI=http://192.168.0.1:11311/
export ROS_IP=192.168.0.1
source /home/ubuntu/catkin_ws/install_isolated/setup.bash
rosparam set use_sim_time false
rostopic echo /cmd_vel

python run_nanocar.py
"""

from lib.nanocar import RosCar
import yaml
import os

def load_config():
    """
    LOAD CONFIG FILE
    Convert config.yml to DICT.
    """
    try:
        FileNotFoundError
    except NameError:
        FileNotFoundError = IOError

    cfg = None
    script_dir = os.path.dirname(os.path.realpath(__file__))
    file_path = os.path.join(script_dir, 'config.yml')
    print(file_path)
    if (os.path.isfile(file_path)):
        with open(file_path, 'r') as ymlfile:
            cfg = yaml.load(ymlfile)
    else:
        raise FileNotFoundError(("File not found: config.yml"))
    return cfg

if __name__ == '__main__':
    cfg = load_config()
    roscar = RosCar(cfg)
    roscar.listener()

