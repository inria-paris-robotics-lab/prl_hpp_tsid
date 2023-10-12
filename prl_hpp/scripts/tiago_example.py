#!/usr/bin/env python
import rospy
rospy.init_node("HPP_example", anonymous=True)
rospy.logwarn("node inited")

from prl_hpp.tiago import planner

# Get current pose
robot, commander_left, commander_right, planner = planner()
commander_right.start_trajectory()

q_curr = robot.get_meas_q()
planner.v(q_curr)

planner.lock_left_arm()
planner.lock_head()
planner.lock_grippers()
planner.lock_torso()

from math import pi
pose = [[0.25, -0.50, 0.6], [0,pi/2.,0]]

path = planner.make_gripper_approach(robot.right_gripper_name, pose, approach_distance = 0.1)
# planner.vf(path)
raw_input("Press any key to play the motion ...")

commander_right.execute_path(path)