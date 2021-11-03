from prl_pinocchio.ur5 import robot
from prl_hpp.planner import Planner
from prl_hpp.commander import Commander

"""
 Create instances of Robot, Planner and Commander classes, dedicated for the Double-UR5 robot.
"""

class UR5_Planner(Planner):
    def __init__(self, robot):
        Planner.__init__(self, robot)

    def lock_grippers(self):
        return self.lock_joints(self.robot.gripper_joints)

    def lock_left_arm(self):
        return self.lock_joints(self.robot.left_arm_joints)

    def lock_right_arm(self):
        return self.lock_joints(self.robot.right_arm_joints)

    def left_gripper_at_pose(self, position, orientation, start_configuration = None):
        return self.tool_at_pose("/left_gripper_grasp_frame", position, orientation, start_configuration)

    def right_gripper_at_pose(self, position, orientation, start_configuration = None):
        return self.tool_at_pose("/right_gripper_grasp_frame", position, orientation, start_configuration)

planner = UR5_Planner(robot)

commander_left_arm = Commander(robot, robot.left_arm_joints, "/left_arm/scaled_vel_joint_traj_controller/follow_joint_trajectory")
commander_right_arm = Commander(robot, robot.right_arm_joints, "/right_arm/scaled_vel_joint_traj_controller/follow_joint_trajectory")
