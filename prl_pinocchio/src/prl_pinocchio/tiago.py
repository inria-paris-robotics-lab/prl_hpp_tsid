import rospy
import xml.etree.ElementTree as ET

from prl_pinocchio.robot import Robot
from prl_pinocchio.commander import Commander
"""
 Create instances of Robot dedicated for the Double-UR5 robot.
"""

class Tiago_Robot(Robot):
    # left_gripper_name = "l_gripper"
    # right_gripper_name = "r_gripper"

    def __init__(self, robot_description_param_prefix, joint_state_topic):
        Robot.__init__(self, robot_description_param_prefix, joint_state_topic)

        # Init joints group
        joints = self.get_joint_names()
        self.gripper_joints = list(filter(lambda joint: joint.lower().find("gripper") != -1, joints))

        # joints = set(joints) - set(self.gripper_joints) # Remove grippers joints from list
        self.left_arm_joints  = list(filter(lambda joint: joint.lower().find("left")  != -1 and joint.lower().find("gripper") == -1, joints))
        self.right_arm_joints = list(filter(lambda joint: joint.lower().find("right") != -1 and joint.lower().find("gripper") == -1, joints))

    # def get_gripper_link(self, gripper):
    #     srdf = ET.fromstring(self.get_srdf_explicit())

    #     gripper = srdf.find(".//gripper[@name='l_gripper']")
    #     if gripper is None:
    #         rospy.logerr(F"Could not find gripper {gripper} in robot srdf")
    #         return None

    #     link = gripper.find("link")
    #     if link is None:
    #         rospy.logwarn(F"No link information found in srdf for gripper {gripper}")
    #         return None

    #     return link.attrib["name"]

def robot():
    rospy.logwarn("Create Tiago Robot...")
    robot = Tiago_Robot("prl_tiago_description", "/joint_states")
    rospy.logwarn("Done")
    return robot


# # Arbitrary value (as velocity and effort limits are already defined in the model)
# robot.MAX_JOINT_ACC = 3.1415926 / 1.0 # 180deg/s^2

# commander_left_arm = Commander(robot, robot.left_arm_joints, trajectory_action_name="/left_arm/scaled_vel_joint_traj_controller/follow_joint_trajectory", fwd_action_name="/left_arm/joint_group_vel_controller")
# commander_right_arm = Commander(robot, robot.right_arm_joints, trajectory_action_name="/right_arm/scaled_vel_joint_traj_controller/follow_joint_trajectory", fwd_action_name="/right_arm/joint_group_vel_controller")