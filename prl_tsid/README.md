prl_tsid
===

# Summary
This package provides "simple" interfaces for using [TSID](https://github.com/stack-of-tasks/tsid/) with ROS robots, aswell as some examples.

The library `prl_tsid` provides mainly one class :
 * [PathFollower](src/prl_hpp/commander.py) : a class that execute a HPP plan using TSID.

It also provides some examples with launchfiles.

# Installation
Dependencies:
 * rospy : http://wiki.ros.org/ROS/Installation
 * tsid : https://github.com/stack-of-tasks/tsid/
 * pinocchio : https://stack-of-tasks.github.io/pinocchio/download.html
 * prl_pinocchio : [link](../prl_pinocchio/README.md)
 * prl_hpp : [link](../prl_hpp/README.md)

# Launch
```
roslaunch prl_tsid sim.launch
```
```
rosrun prl_tsid example_hpp.py
```
