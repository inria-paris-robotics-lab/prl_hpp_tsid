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
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Joy

tf_listener = tf.TransformListener()

# All this part is erroneous in most case #
# Temporary fix until pin is released #
def is_frame_supported(model, parent_frame_id, child_frame_id):
    current_id = model.frames[child_frame_id].previousFrame
    while model.frames[parent_frame_id].parent == model.frames[current_id].parent and  current_id > parent_frame_id:
        current_id = model.frames[current_id].previousFrame
    return model.frames[parent_frame_id].parent == model.frames[current_id].parent and current_id == parent_frame_id

# Are we sure about this property ? => child_frame_id > parent_frame_id
def compute_supported_frames_in_body(model, parent_frame_id):
    res = []
    for i,f in enumerate(model.frames[parent_frame_id:], start=parent_frame_id):
        if(is_frame_supported(model, parent_frame_id, i)):
            res.append(i)
    return res

def compute_supported_inertia_in_body(model, frame_id):
    supported_frames = [frame_id] + compute_supported_frames_in_body(model,frame_id)
    j_inertia = pin.Inertia(0, np.zeros(3), np.zeros([3,3]))
    for f_id in supported_frames:
        jMf = model.frames[f_id].placement
        j_inertia += jMf * model.frames[f_id].inertia # Compute the inertia in the joint frame
    jMf = model.frames[frame_id].placement
    return jMf.actInv(j_inertia)

def compute_direct_child_joint(model, frame_id):
    parent_joint_id = model.frames[frame_id].parent
    child_joints = model.subtrees[parent_joint_id]
    direct_child_joints = []
    for j_id in child_joints:
        if(model.parents[j_id] == parent_joint_id):
            direct_child_joints.append(j_id)
    return direct_child_joints

def compute_supported_inertia_crba(model, data, frame_id):
    oMf = pin.updateFramePlacement(model, data, frame_id)
    o_Inertia = oMf * compute_supported_inertia_in_body(model, frame_id)

    joints = compute_direct_child_joint(model, frame_id)
    for j_id in joints:
        oMj = data.oMi[j_id]
        joint_crb_inertia = data.Ycrb[j_id]
        o_Inertia += oMj * joint_crb_inertia # Sum everything at origin for simplicity
    return oMf.actInv(o_Inertia)

def compute_supported_effort(model, data, frame_id):
    inertia = compute_supported_inertia_crba(model, data, frame_id)
    v_frame = pin.getFrameVelocity(model, data, frame_id, pin.ReferenceFrame.LOCAL)
    a_frame = pin.getFrameAcceleration(model, data, frame_id, pin.ReferenceFrame.LOCAL)
    effort = inertia.vxiv(v_frame) + inertia * a_frame # Is it doing the right thing ?
    effort -= inertia * model.gravity # Is it the right sign ?
    return effort
###################################

def control_from_fts_cb(msg):
    max_speed = 0.2
    max_rot = np.pi/2

    frame_id = robot.pin_robot_wrapper.model.getFrameId("left_measurment_joint")
    t, q, v, tau = robot.get_meas_qvtau()
    pin.aba(robot.pin_robot_wrapper.model, robot.pin_robot_wrapper.data, np.array(q), np.array(v), np.array(tau))
    pin.crba(robot.pin_robot_wrapper.model, robot.pin_robot_wrapper.data, np.array(q))
    effort = compute_supported_effort(robot.pin_robot_wrapper.model, robot.pin_robot_wrapper.data, frame_id)

    wrist_f_tot = pin.Force(np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]))
    diff_f = effort + wrist_f_tot

    answer = WrenchStamped(header = msg.header)

    answer.wrench.force.x = diff_f.linear[0]
    answer.wrench.force.y = diff_f.linear[1]
    answer.wrench.force.z = diff_f.linear[2]

    answer.wrench.torque.x = diff_f.angular[0]
    answer.wrench.torque.y = diff_f.angular[1]
    answer.wrench.torque.z = diff_f.angular[2]

    pub.publish(answer)

    # print("computed inertia", inertia)
    # print("computed effort", effort)
    # print("measured effort", wrist_f_tot)

    # # q, v, tau = robot.get_meas_qvtau()
    # # wrist_f_tot = pin.Force(np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]))
    # # wrist_f_grav = 

    # # ee_vel_vec = (baseMgripper * pin.Motion(ee_vel_vec_local)).vector
    # # ee_pos_vec = np.concatenate((baseMgripper.translation, baseMgripper.rotation.flatten('F')))

    # vz = 0.01 * ((-5.) - msg.wrench.force.z)
    # v_z = np.clip(vz, -max_speed, max_speed)
    # ee_vel_vec = np.array([0,0,v_z,0,0,0])
    # pf.eeSample.derivative(ee_vel_vec)
    # # pf.eeSample.value(ee_pos_vec)

def control_from_joy_cb(msg):
    max_speed = 0.2
    max_rot = np.pi/2

    print()
    ee_vel_vec_local = np.array([max_speed*msg.axes[6], max_speed*msg.axes[7], max_speed*msg.axes[1], 0, 0, max_rot*msg.axes[3]])
    # print("Cmd velocity : ", ee_vel_vec_local)

    pf.eeVelSample.derivative(ee_vel_vec_local)

if __name__=='__main__':
    pub = rospy.Publisher("/left_ft_wrench_error", WrenchStamped, queue_size=0)
    # rospy.Subscriber("/left_ft_wrench", WrenchStamped, control_from_fts_cb)
    rospy.Subscriber("/joy", Joy, control_from_joy_cb)

    start_pose = [[-0.6, 0.0, 0.05], [np.pi, 0, np.pi]]
    path = planner.make_gripper_approach(robot.left_gripper_name, start_pose, approach_distance = 0.01)

    input("Press enter to execute initial motion...")
    pf.execute_path(path, [commander_left_arm], 0.1, velocity_ctrl = True)

    robot.remove_collision_pair("table_link_0", "left_gripper_finger_1_finger_tip_0")
    robot.remove_collision_pair("table_link_0", "left_gripper_finger_1_finger_tip_1")
    robot.remove_collision_pair("table_link_0", "left_gripper_finger_1_flex_finger_0")
    robot.remove_collision_pair("table_link_0", "left_gripper_finger_2_finger_tip_0")
    robot.remove_collision_pair("table_link_0", "left_gripper_finger_2_finger_tip_1")
    robot.remove_collision_pair("table_link_0", "left_gripper_finger_2_flex_finger_0")

    input("Press enter to execute TSID motion...")
    pf.follow_velocity("left_gripper_grasp_frame", [commander_left_arm], 0.1, velocity_ctrl = True)

    input("Press enter to quit...")