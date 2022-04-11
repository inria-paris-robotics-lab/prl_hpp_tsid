#!/usr/bin/env python

# roslaunch aruco_detect aruco_detect.launch camera:=left_camera/color image:=image_rect_color fiducial_len:=0.08 dictionary:=5

from prl_tsid.commander import PathFollower
import numpy as np
import pinocchio as pin

import rospy
rospy.init_node("TSID_example", anonymous=True)
rospy.logwarn("init_node")

# Plan a trajectory using HPP
from prl_hpp.ur5 import planner, robot, commander_left_arm, commander_right_arm
pi = 3.1415926
rospy.logwarn("imported")

# Start the commanders
commander_left_arm.start_fwd()
commander_right_arm.start_fwd()

planner.lock_grippers()
planner.lock_right_arm()
planner.set_velocity_limit(0.25)
planner.set_acceleration_limit(0.25)

# Exectute the trajectories using TSID
pf = PathFollower(robot)
pf.set_velocity_limit(0.5)
pf.set_torque_limit(0.75)
pf.set_acceleration_limit(0.1)

import tf
tf_listener = tf.TransformListener()
def fid_cb(msg):
    fid = None
    for fiducial in msg.transforms:
        if(fiducial.fiducial_id == 0):
            fid = fiducial
            break

    if not fid:
        return

    tf_listener.waitForTransform("prl_ur5_base", msg.header.frame_id, msg.header.stamp, rospy.Duration(4.0))
    camera_pose = tf_listener.lookupTransform("prl_ur5_base", msg.header.frame_id, msg.header.stamp)

    baseMcamera = pin.XYZQUATToSE3(camera_pose[0] + camera_pose[1])
    cameraMfid = pin.XYZQUATToSE3([fid.transform.translation.x, fid.transform.translation.y, fid.transform.translation.z, fid.transform.rotation.x, fid.transform.rotation.y, fid.transform.rotation.z, fid.transform.rotation.w])
    baseMfid = baseMcamera * cameraMfid

    gripperMgoal = pin.XYZQUATToSE3([.0,.0,.5, 0,0,0,1])

    # We give the fid the same rotation as the gripper
    baseMgripper = pin.XYZQUATToSE3([0,0,0, 0.5,0.5,0.5,0.5])
    baseMfid.rotation =  baseMgripper.rotation

    baseMgoal = baseMfid * gripperMgoal.inverse()

    ee_pos_vec = np.concatenate((baseMgoal.translation, baseMgoal.rotation.flatten('F')))

    pf.eeSample.value(ee_pos_vec)


from fiducial_msgs.msg import FiducialTransformArray
rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, fid_cb)
rospy.logwarn("cb registered")

start_pose = [[-0.4, 0, 0.2], [0.5,0.5,0.5,0.5]]
path = planner.make_gripper_approach(robot.left_gripper_name, start_pose, approach_distance = 0.01)
path.targetFrames.append("right_gripper_grasp_frame")

input("Press enter to execute initial motion")
pf.execute_path(path, [commander_left_arm, commander_right_arm], 0.1, True)


input("Press enter to execute TSID motion")
pf.follow_velocity("left_gripper_grasp_frame", [commander_left_arm, commander_right_arm], 0.1, True)
