import rospy

from prl_pinocchio.tiago import robot_commanders
from prl_hpp.planner import Planner

"""
 Create instances of Robot, Planner and Commander classes, dedicated for the Double-UR5 robot.
"""

class Tiago_Planner(Planner):
    def __init__(self, robot):
        Planner.__init__(self, robot)

    def lock_grippers(self):
        return self.lock_joints(self.robot.gripper_joints)

    def lock_left_arm(self):
        return self.lock_joints(self.robot.left_arm_joints)

    def lock_right_arm(self):
        return self.lock_joints(self.robot.right_arm_joints)

    def lock_head(self):
        return self.lock_joints(self.robot.head_joints)

    def lock_torso(self):
        return self.lock_joints(self.robot.torso_joints)

    # def left_gripper_at_pose(self, position, orientation, start_configuration = None):
    #     return self.tool_at_pose("/left_gripper_grasp_frame", position, orientation, start_configuration)

    # def right_gripper_at_pose(self, position, orientation, start_configuration = None):
    #     return self.tool_at_pose("/right_gripper_grasp_frame", position, orientation, start_configuration)

def planner():
    r, c_left, c_right = robot_commanders()
    planner = Tiago_Planner(r)
    return r, c_left, c_right, planner