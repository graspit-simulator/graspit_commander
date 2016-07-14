graspit_commander
=================
Python ROS Client for GraspIt!

This package offers a python client for the services exposed in:
https://github.com/CURG/graspit_interface

Installing GraspIt! Commander
----------------------------
Follow the instructions in the GraspIt Interface [README](https://github.com/CURG/graspit_interface)
to install GraspIt! and the GraspitInterface plugin.

Clone this repository into your graspit workspace:
```
cd graspit_ros_ws/src
git clone git@github.com:CURG/graspit_commander.git
```

Run `catkin_make` from the root of your workspace (just like in the GraspitInterface readme) to 
install all of the python dependencies and add GraspitCommander to your path.


Using GraspIt! Commander
------------------------
Before running any python scripts that use the GraspitCommander, run `source devel/setup.zsh` from
the root of your GraspIt! workspace to add the GraspitCommander to your python path. 

Then, in any python program you can simply import from GraspitCommander
i.e.
```
from graspit_commander import GraspitCommander
GraspitCommander.planGrasps(max_steps=50000)
```

Running Tests
-------------
We use py.test to run all of our integration tests. It should automatically be installed when you
first run catkin_make in this workspace.

Run `py.test` from the root of this package to run all tests (the -v option gives you verbose output)

