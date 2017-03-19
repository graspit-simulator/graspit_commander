graspit_commander
=================
Python ROS Client for GraspIt!

This package offers a python client for the services exposed in:
https://github.com/graspit-simulator/graspit_interface

Installation:
----------------------------
Follow the instructions at [https://github.com/graspit-simulator/graspit_interface](https://github.com/graspit-simulator/graspit_interface).

Usage:
------------------------
Start GraspIt! interface in one terminal.
```
source devel/setup.bash
roslaunch graspit_interface graspit_interface.launch
```

Then in a second terminal source ros workspace:
```
source devel/setup.bash
```

Then in python:
```
from graspit_commander import GraspitCommander
GraspitCommander.loadWorld("plannerMug")
GraspitCommander.planGrasps(max_steps=50000)
```

Tests:
-------------
In one terminal:
```
source devel/setup.bash
roslaunch graspit_interface graspit_interface.launch
```
Then in a second terminal:
```
source devel/setup.bash
roscd graspit_commander
py.test
```
