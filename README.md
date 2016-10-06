graspit_commander
=================
Python ROS Client for GraspIt!

This package offers a python client for the services exposed in:
https://github.com/CURG/graspit_interface

Installing GraspIt! Commander
----------------------------
Follow the instructions in the GraspIt Interface [README](https://github.com/CURG/graspit_interface)
to create a ros workspace with this package.


Using GraspIt! Commander
------------------------
```
//first source ros workspace
source devel/setup.bash
```

Then in python or IPython
```
from graspit_commander import GraspitCommander
GraspitCommander.loadWorld("plannerMug")
GraspitCommander.planGrasps(max_steps=50000)
```

Running Tests
-------------
We use py.test to run all of our integration tests. It should automatically be installed when you
first run catkin_make in this workspace.

Run `py.test` from the root of this package to run all tests (the -v option gives you verbose output)

