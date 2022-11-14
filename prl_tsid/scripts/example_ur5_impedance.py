#!/usr/bin/env python

# roslaunch prl_ur5_run real.launch velocity_control:=true ff_control:=true moveit:=false sensors:=true enable_right_camera:=false
# roslaunch ros_cosypose singleview_loop.launch bringup_camera:=false dataset:=ycbv debug:=true camera_name:=left_camera/color detection_threshold:=0.85

from prl_tsid.commander import PathFollower
import numpy as np
import pinocchio as pin

import rospy
rospy.init_node("TSID_example", anonymous=True)
rospy.loginfo("init_node")

# Plan a trajectory using HPP
from prl_hpp.ur5 import planner, robot, commander_left_arm, commander_right_arm
pi = 3.1415926

# Start the commanders
commander_left_arm.start_fwd()

planner.lock_grippers()
planner.lock_right_arm()
planner.set_velocity_limit(0.25)
planner.set_acceleration_limit(0.25)

# Exectute the trajectories using TSID
pf = PathFollower(robot)
pf.set_velocity_limit(1)
pf.set_acceleration_limit(1)
pf.set_torque_limit(0.75)

import tf
tf_listener = tf.TransformListener()
def control_cb(msg):
    max_speed = 0.2
    max_rot = np.pi/2
    gripper_pose = tf_listener.lookupTransform("prl_ur5_base", "left_gripper_grasp_frame", rospy.Time(0))
    baseMgripper = pin.XYZQUATToSE3(gripper_pose[0] + gripper_pose[1])

    print()
    ee_vel_vec_local = np.array([max_speed*msg.axes[6], max_speed*msg.axes[7], max_speed*msg.axes[1], 0, 0, max_rot*msg.axes[3]])
    print("Cmd velocity : ", ee_vel_vec_local)

    ee_vel_vec = ee_vel_vec_local
    # ee_vel_vec = (baseMgripper * pin.Motion(ee_vel_vec_local)).vector
    print("Computed vel : ", ee_vel_vec)

    ee_pos_vec = np.concatenate((baseMgripper.translation, baseMgripper.rotation.flatten('F')))

    pf.eeSample.derivative(ee_vel_vec)
    pf.eeSample.value(ee_pos_vec)


from sensor_msgs.msg import Joy
rospy.Subscriber("/joy", Joy, control_cb)

start_pose = [[-0.5, 0.1, 0.05], [np.pi, 0, np.pi]]
path = planner.make_gripper_approach(robot.left_gripper_name, start_pose, approach_distance = 0.01)

input("Press enter to execute initial motion")
pf.execute_path(path, [commander_left_arm], 0.1, velocity_ctrl = True)


input("Press enter to execute TSID motion")
pf.follow_velocity("left_gripper_grasp_frame", [commander_left_arm], 0.1, velocity_ctrl = True, Kp_ee = 0, Kd_ee = 6.0)
