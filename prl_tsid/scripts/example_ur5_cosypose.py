#!/usr/bin/env python

# roslaunch prl_ur5_run real.launch velocity_control:=true ff_control:=true moveit:=false sensors:=true enable_right_camera:=false
# roslaunch ros_cosypose singleview_loop.launch bringup_camera:=false dataset:=ycbv debug:=true camera_name:=left_camera/color detection_threshold:=0.85

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
def cosy_cb(msg):
    bottle = None
    for obj in msg.objects:
        if(obj.label == "obj_000014"):
            bottle = obj
            break

    if not bottle:
        return

    tf_listener.waitForTransform("prl_ur5_base", msg.header.frame_id, msg.header.stamp, rospy.Duration(4.0))
    camera_pose = tf_listener.lookupTransform("prl_ur5_base", msg.header.frame_id, msg.header.stamp)

    baseMcamera = pin.XYZQUATToSE3(camera_pose[0] + camera_pose[1])
    cameraMbottle = pin.XYZQUATToSE3([bottle.pose.position.x, bottle.pose.position.y, bottle.pose.position.z, bottle.pose.orientation.x, bottle.pose.orientation.y, bottle.pose.orientation.z, bottle.pose.orientation.w])
    baseMbottle = baseMcamera * cameraMbottle

    gripperMgoal = pin.XYZQUATToSE3([.0,.025,.30, 0,0,0,1])

    # We give the bottle the same rotation as the gripper
    baseMgripper = pin.XYZQUATToSE3([0,0,0, 0.5,0.5,0.5,0.5])
    baseMbottle.rotation =  baseMgripper.rotation

    baseMgoal = baseMbottle * gripperMgoal.inverse()

    ee_pos_vec = np.concatenate((baseMgoal.translation, baseMgoal.rotation.flatten('F')))

    pf.eeSample.value(ee_pos_vec)


from ros_cosypose.msg import CosyObjectArray
rospy.Subscriber("/cosyobject_list", CosyObjectArray, cosy_cb)
rospy.logwarn("cb registered")

start_pose = [[-0.4, 0, 0.2], [0.5,0.5,0.5,0.5]]
path = planner.make_gripper_approach(robot.left_gripper_name, start_pose, approach_distance = 0.01)
path.targetFrames.append("right_gripper_grasp_frame")

input("Press enter to execute initial motion")
pf.execute_path(path, [commander_left_arm, commander_right_arm], 0.1, True)


input("Press enter to execute TSID motion")
pf.follow_velocity("left_gripper_grasp_frame", [commander_left_arm, commander_right_arm], 0.1, True)
