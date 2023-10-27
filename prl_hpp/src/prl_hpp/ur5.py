import rospy

from prl_pinocchio.ur5 import robot, commander_left_arm, commander_right_arm
from prl_hpp.planner import Planner

"""
 Create instances of Robot, Planner and Commander classes, dedicated for the Double-UR5 robot.
"""

class UR5_Planner(Planner):
    def __init__(self, robot):
        Planner.__init__(self, robot)

        # Disable some collision margin checks
        # The two firsts joint of the UR are very close to the base, but the chances of a collision is very low. No margin is applied.
        self._collision_margin_exclusion += [
            ('robot/left_shoulder_lift_joint', 'universe'),
            ('robot/left_shoulder_pan_joint', 'universe'),
            ('robot/right_shoulder_lift_joint', 'universe'),
            ('robot/right_shoulder_pan_joint', 'universe'),
        ]

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
