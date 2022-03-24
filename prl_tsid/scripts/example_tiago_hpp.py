#!/usr/bin/env python
import rospy
rospy.init_node("TSID_example", anonymous=True)

# Plan a trajectory using HPP
from prl_hpp.tiago import planner
from prl_tsid.commander import PathFollower

robot, commander_left, commander_right, planner = planner()

q_curr = robot.get_meas_q()
planner.v(q_curr)
planner.lock_left_arm()
planner.lock_head()
planner.lock_grippers()
planner.lock_torso()

planner.set_velocity_limit(0.25)
planner.set_acceleration_limit(0.25)

pi = 3.1415926
pose = [[0.25, -0.50, 0.6], [0,pi/2.,0]]

path = planner.make_gripper_approach(robot.right_gripper_name, pose, approach_distance = 0.1)

# Start the commanders
# commander_left_arm.start_fwd()
# commander_right_arm.start_fwd()
commander_left.start_trajecotry_cmd()
commander_right.start_trajecotry_cmd()

# Exectute the trajectories using TSID
pf = PathFollower(robot)
pf.set_velocity_limit(1)
pf.set_torque_limit(1)

# path.targetFrames.append(robot.get_gripper_link(robot.right_gripper_name))
raw_input("Press enter to execute path")
# pf.execute_path(path, [commander_right, commander_left], 0.001)
pf.v = planner.v
pf.execute_path(path, [commander_right], 0.005)