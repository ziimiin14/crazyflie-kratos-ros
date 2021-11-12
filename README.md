[![Build Status](https://travis-ci.org/whoenig/crazyflie_ros.svg?branch=master)](https://travis-ci.org/whoenig/crazyflie_ros)

> **WARNING**: This repository is deprecated. Please use the [Crazyswarm](http://crazyswarm.readthedocs.io) instead, even when only operating a single Crazyflie. If you are missing features in the Crazyswarm, feel free to [open an open issue](https://github.com/USC-ACTLab/crazyswarm/issues).

crazyflie_ros
=============

ROS stack for Bitcraze Crazyflie (http://www.bitcraze.se/), with the following features:

* Support for Crazyflie 1.0 and Crazyflie 2.0 (using stock firmware)
* Publishes on-board sensors in ROS standard message formats
* Supports ROS parameters to reconfigure crazyflie parameters
* Support for using multiple Crazyflies with a single Crazyradio
* Includes external controller for waypoint navigation (if motion capture system is available)
* No dependency to the Bitcraze SDK (Driver and Controller written in C++)

A tutorial (for a slightly older version) is available in W. Hönig and N. Ayanian. "Flying Multiple UAVs Using ROS", Chapter in Robot Operating System (ROS): The Complete Reference (Volume 2), Springer, 2017. (see http://act.usc.edu/publications.html for a free pre-print).

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

## Installation

Clone the package into your catkin workspace:
```
git clone https://github.com/whoenig/crazyflie_ros.git
cd crazyflie_ros
git submodule init
git submodule update
```

Use `catkin_make` on your workspace to compile.

## Run

### Crazyflie_demo

This package contains a rich set of examples to get quickly started with the Crazyflie bolt for kratos

For teleoperation using a joystick, use:
```
roslaunch crazyflie_demo vrpn_teleop_turningy.launch server:=192.168.137.1
```
