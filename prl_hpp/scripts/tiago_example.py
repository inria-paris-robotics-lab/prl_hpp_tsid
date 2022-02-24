#!/usr/bin/env python
import rospy
rospy.init_node("HPP_example", anonymous=True)
rospy.logwarn("node inited")

from prl_hpp.tiago import robot, planner #, commander_left_arm, commander_right_arm

# Get current pose
robot, planner = planner()
q_curr = robot.get_meas_q()
planner.v(q_curr)