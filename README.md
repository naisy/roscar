# ROSCAR


## ROSCAR
### on Jetson TX2(car)
[![TX2](https://img.youtube.com/vi/FSd1MBqDlXk/1.jpg)](https://www.youtube.com/watch?v=FSd1MBqDlXk)

#### Requirement
```
sudo apt-get install ros-kinetic-desktop-full
sudo pip install smbus2
```
IP_Address: 192.168.0.56<br>
```
sudo su
export ROS_MASTER_URI=http://192.168.0.56:11311/
export ROS_IP=192.168.0.56
roscore&
python run_roscar.py
```

### on PC(controller)
#### Requirement
```
sudo apt-get install ros-kinetic-desktop-full
sudo apt-get install python-pygame
```
Set CAR IP_Address: 192.168.0.56<br>
```
export ROS_MASTER_URI=http://192.168.0.56:11311/
export ROS_IP=192.168.0.56
python joycontrol.py
```

## AutowareCar
[![TX2](https://img.youtube.com/vi/bYBoUJiRxcw/1.jpg)](https://www.youtube.com/watch?v=bYBoUJiRxcw)

### Requirement
* [Autoware](https://github.com/naisy/Autoware/tree/master/docker/JetsonTX2)
```
sudo pip install smbus2
```
### on car
```
sudo su
python run_autoware_car.py
```
