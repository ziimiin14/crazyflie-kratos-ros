[![Build Status](https://travis-ci.org/whoenig/crazyflie_ros.svg?branch=master)](https://travis-ci.org/whoenig/crazyflie_ros)

crazyflie ros for kratos
=============

## Installation

Clone the package into your catkin workspace:
```
mkdir catkin_ws_crazyflie
cd catkin_ws_crazyflie
mkdir src
cd src
git clone https://github.com/ziimiin14/crazyflie-kratos-ros.git
cd crazyflie-kratos-ros
git submodule init
git submodule update
cd ~/catkin_ws_crazyflie
catkin init
catkin_make
```

Use `catkin_make` on your workspace to compile.

## Run

### Crazyflie_demo

This package contains a rich set of examples to get quickly started with the Crazyflie bolt for kratos

For teleoperation using a joystick, use:
```
roslaunch crazyflie_demo vrpn_teleop_turningy.launch server:=192.168.137.1
```
P/S: If kratos drifts a ton, you are able to tune down the kp value from 6550 to 3275 in this [launch file](https://github.com/ziimiin14/crazyflie-kratos-ros/blob/master/crazyflie_demo/launch/turningy.launch). The kp value refers to the p gain value for the flaps control.
## Scripts
Refers to this [scripts folder](https://github.com/ziimiin14/crazyflie-kratos-ros/tree/master/scripts)

[rosbag_readDataExtMod.py](https://github.com/ziimiin14/crazyflie-kratos-ros/blob/master/scripts/rosbag_readDataExtMod.py) reads the rosbag file recorded 'DataExtMod' raw data.
[rosbag_readEventStruct.py](https://github.com/ziimiin14/crazyflie-kratos-ros/blob/master/scripts/rosbag_readEventStruct.py) reads the rosbag file recorded 'eventStruct' msg recorded from event camera and turn the raw data into bin files.

To run rosbag_readEventStruct.py
```
python3 rosbag_readEventStruct.py ROSBAG_PATH --time_output_file=DESIRED_TIME_BIN_FILE_PATH --event_output_file=DESIRED_EVENT_BIN_FILE_PATH
```
## Citing This Work

This project is published under the very permissive MIT License. However,
if you use the package we appreciate if you credit this project accordingly.

For academic publications, you can cite the following book chapter:
```
@Inbook{crazyflieROS,
  author={Wolfgang H{\"o}nig
          and Nora Ayanian},
  editor={Anis Koubaa},
  title={Flying Multiple UAVs Using ROS},
  bookTitle={Robot Operating System (ROS): The Complete Reference  (Volume 2)},
  year={2017},
  publisher={Springer International Publishing},
  pages={83--118},
  isbn={978-3-319-54927-9},
  doi={10.1007/978-3-319-54927-9_3},
  url={https://doi.org/10.1007/978-3-319-54927-9_3}
}

```

If your work is related to Mixed Reality, you might cite the paper which introduced the package instead, using the following bibtex entry:
```
@conference{HoenigMixedReality2015,
  author = {Wolfgang H{\"o}nig and Christina Milanes and Lisa Scaria and Thai Phan and Mark Bolas and Nora Ayanian},
  booktitle = {IEEE/RSJ Intl Conf. Intelligent Robots and Systems},
  pages = {5382 - 5387},
  title = {Mixed Reality for Robotics},
  year = {2015}}
```

For any other mentioning please include my affiliation (ACTLab at University of Southern California or USC in short; The link to our webpage is http://act.usc.edu) as this work was partially done as part of my research at USC.
