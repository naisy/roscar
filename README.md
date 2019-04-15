# ROSCAR


## ROSCAR
### on Jetson TX2(car)
#### Requirement
```
sudo apt-get install ros-kinetic-desktop-full
sudo pip install smbus2
```
terminal 1:<br>
```
sudo su
export ROS_MASTER_URI=http://192.168.0.56:11311/
export ROS_IP=192.168.0.56
python run_roscar.py
```

### on PC(controller)
#### Requirement
```
sudo apt-get install ros-kinetic-desktop-full
sudo apt-get install python-pygame
```
```
export ROS_MASTER_URI=http://192.168.0.56:11311/
export ROS_IP=192.168.0.56
python joycontrol.py
```

## AutowareCar
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
