#!/usr/bin/env python
import rospy
rospy.init_node("HPP_example", anonymous=True)
rospy.logwarn("node inited")

from prl_hpp.tiago import planner

# Get current pose
robot, commander_left, commander_right, planner = planner()

q_curr = robot.get_meas_q()
planner.v(q_curr)

planner.lock_left_arm()
planner.lock_head()
planner.lock_grippers()

from math import pi
pose = [[0.5, 0.00, 0.6], [pi,0,0]]

paths = planner.make_gripper_approach(robot.right_gripper_name, pose, approach_distance = 0.2)