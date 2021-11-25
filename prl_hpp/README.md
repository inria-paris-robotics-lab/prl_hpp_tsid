prl_hpp
===

# Summary
This package provides "simple" interfaces for using [HPP](https://humanoid-path-planner.github.io/hpp-doc/) with ROS robots, aswell as some examples.

The library `prl_hpp` provides mainly two classes :
 * [Planner](src/prl_hpp/planner.py) : Formulate and solve some classic path planning problems (using [HPP](https://humanoid-path-planner.github.io/hpp-doc/)) and return the plans.
 * [Commander](src/prl_hpp/commander.py) : a class that allows to send commands to the robot for following a hpp plans. (It is built on top of the [prl_pinocchio](../prl_pinocchio/README.md) Commander)

It also provides some files to instantiate all the above class for the prl robots setup :
 * [ur5.py](src/prl_hpp/ur5.py)

And some examples with launchfiles.

# Installation
Dependencies:
 * rospy : http://wiki.ros.org/ROS/Installation
 * hpp-manipulation-corba, hpp-gepetto-viewer : https://humanoid-path-planner.github.io/hpp-doc/download.html
 * prl_pinocchio : [link](../prl_pinocchio/README.md)

(optionnal)
 * prl_ur5_robot : https://github.com/inria-paris-robotic-lab/prl_ur5_robot

# Launch
```
roslaunch prl_hpp sim.launch
```
```
rosrun prl_hpp example.py
```
