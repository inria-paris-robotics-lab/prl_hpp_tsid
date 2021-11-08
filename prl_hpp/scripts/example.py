#!/usr/bin/env python
import rospy

rospy.init_node("HPP", anonymous=True)

from prl_hpp.ur5 import robot, planner, commander_left_arm as commander_L, commander_right_arm as commander_R
pi = 3.1415926

planner.lock_grippers()
planner.lock_right_arm()
planner.set_velocity_limit(0.25)
planner.set_acceleration_limit(0.25)

# Get current pose
# q_curr = [-1.56679958631439, -1.5684298666798968, -1.5687736924513271, 0.00526034337325143, 1.580046563493707, -0.7848526556848037, 0.0005266352836788002, 0.0005266352836788002, 0.0005266352836788002, 0.0005266352836788002, 0.0005266352836788002, 0.0005266352836788002, 0.0005266352836788002, 0.0005266352836788002, 1.5666911247430821, -1.5716259708390847, 1.568915751241894, 3.133750783715004, -1.5819195534407946, 0.7839731336070219, 5.51347657511414e-05, 5.51347657511414e-05, 5.51347657511414e-05, 5.51347657511414e-05, 5.51347657511414e-05, 5.51347657511414e-05]
q_curr = robot.get_meas_q()
planner.v(q_curr)


pose_1 = [[-0.50, 0, 0.1], [0, -pi/2, 0]]
pose_2 = [[-0.40, 0, 0.1], [0, -pi/2, 0]]

paths = planner.make_pick_and_place(robot.left_gripper_name, pose_1, pose_2, approach_distance = 0.2)

# input("Solution found. Press enter to display path")
# planner.pp(paths[0].id)
# input("Press enter to display path")
# planner.pp(paths[1].id)
# input("Press enter to display path")
# planner.pp(paths[2].id)

input("Solution found. Press enter to execute path")
commander_L.execute(paths[0])
commander_L.execute(paths[1])
commander_L.execute(paths[2])