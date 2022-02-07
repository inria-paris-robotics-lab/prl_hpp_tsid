#!/usr/bin/env python
import rospy
rospy.init_node("HPP_example", anonymous=True)

from prl_hpp.ur5 import robot, planner, commander_left_arm, commander_right_arm
pi = 3.1415926

planner.lock_grippers()
planner.lock_right_arm()
planner.set_velocity_limit(0.25)
planner.set_acceleration_limit(0.25)

# Get current pose
q_curr = robot.get_meas_q()
planner.v(q_curr)

pose_1 = [[-0.20, 0.00, 0.1], [pi,0,0]]
pose_2 = [[-0.50, 0.30, 0.1], [pi,0,0]]

paths = planner.make_pick_and_place(robot.left_gripper_name, pose_1, pose_2, approach_distance = 0.2)

print(paths)
# input("Solution found. Press enter to display path")
# planner.pp(paths[0].id)
# # input("Press enter to display path")
# planner.pp(paths[1].id)
# # input("Press enter to display path")
# planner.pp(paths[2].id)

# input("Solution found. Press enter to execute path")
# commander_left_arm.execute_path(paths[0])
# commander_left_arm.execute_path(paths[1])
# commander_left_arm.execute_path(paths[2])