prl_pinocchio
===

# Summary
This package make the bridge between the robot ROS interfaces and the concept of a robot for pinocchio.

The library `prl_pinocchio` provides mainly two classes :
 * [Commander](src/prl_pinocchio/commander.py) : a class that allows to send commands to the robot.
 * [Robot](src/prl_pinocchio/robot.py) : a class that creates the pinoccchio model of the robot, allows to read the state of the ROS robot, etc ...

It also provides some files to instantiate all the above class for the prl robots setup :
 * [ur5.py](src/prl_pinocchio/ur5.py)

And some launchfiles.

**Remark**:  
  All the joint configurations, velocities, accelerations, ... are expressed in the joint order of the pinocchio model. The above classes will handle the transformations to/from the ROS robot.

# Installation
Dependencies:
 * rospy : http://wiki.ros.org/ROS/Installation
 * pinocchio : https://stack-of-tasks.github.io/pinocchio/download.html