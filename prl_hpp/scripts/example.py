#!/usr/bin/env python3
import rospy
rospy.init_node("HPP_example", anonymous=True)

from prl_hpp.ur5 import robot, planner, commander_left_arm, commander_right_arm
pi = 3.1415926

planner.lock_grippers()
planner.lock_right_arm()
planner.set_velocity_limit(0.25)
planner.set_acceleration_limit(0.25)

commander_left_arm.start_trajectory()

# Get current pose
# q_curr = robot.get_meas_q()
# planner.v(q_curr)

pose_1 = [[-0.20, 0.00, 0.4], [pi/2, 0, pi/2]]
pose_2 = [[-0.40, 0.20, 0.4], [pi/2, 0, pi]]

path = planner.make_gripper_approach(robot.left_gripper_name, pose_1, approach_distance = 0.2)
input("Solution found. Press enter to display path")
planner.pp(path.id)
input("Solution found. Press enter to execute path")
commander_left_arm.execute_path(path)

# paths = planner.make_pick_and_place(robot.left_gripper_name, pose_1, pose_2, approach_distance = 0.2)

# input("Solution found. Press enter to display paths")
# planner.pp(paths[0].id)
# planner.pp(paths[1].id)
# planner.pp(paths[2].id)

