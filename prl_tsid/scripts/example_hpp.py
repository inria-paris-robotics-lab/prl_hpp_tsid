from prl_tsid.commander import PathFollower

import rospy
rospy.init_node("TSID_example", anonymous=True)

# Plan a trajectory using HPP
from prl_hpp.ur5 import planner, robot, commander_left_arm, commander_right_arm
pi = 3.1415926

planner.lock_grippers()
planner.lock_right_arm()
planner.set_velocity_limit(0.25)
planner.set_acceleration_limit(0.25)

pose_1 = [[-0.50, 0, 0.1], [pi, 0, 0]]
pose_2 = [[-0.40, 0, 0.1], [pi, 0, 0]]

paths = planner.make_pick_and_place(robot.left_gripper_name, pose_1, pose_2, approach_distance = 0.2)

# Start the commanders
commander_left_arm.start_fwd()
commander_right_arm.start_fwd()
# commander_left_arm.start_trajecotry()
# commander_right_arm.start_trajecotry()

# Exectute the trajectories using TSID
pf = PathFollower(robot)
pf.set_velocity_limit(0.25)
pf.set_torque_limit(0.5)

paths[0].targetFrames.append("right_gripper_grasp_frame")
input("Press enter to execute path")
pf.execute_path(paths[0], [commander_left_arm, commander_right_arm], 0.02)

