import rospy

import pinocchio as pin
from sensor_msgs.msg import JointState
from prl_pinocchio.tools.observer import Observer
from prl_pinocchio.robot import Robot
from prl_pinocchio.commander import Commander
"""
 Create instances of Robot dedicated for the Double-UR5 robot.
"""

class Tiago_Robot(Robot):
    left_gripper_name = "l_gripper"
    right_gripper_name = "r_gripper"

    def __init__(self, robot_description_param_prefix, joint_state_topic):
        Robot.__init__(self, robot_description_param_prefix)

        self.joint_state_topic = joint_state_topic
        self._joint_state_obs = Observer(joint_state_topic, JointState)

        # Init joints group
        joints = self.get_joint_names()
        self.gripper_joints = list(filter(lambda joint: joint.lower().find("gripper") != -1 or joint.lower().find("hand") != -1, joints))
        self.left_arm_joints  = list(filter(lambda joint: joint.lower().find("arm_left")  != -1, joints))
        self.right_arm_joints = list(filter(lambda joint: joint.lower().find("arm_right") != -1, joints))
        self.head_joints = list(filter(lambda joint: joint.lower().find("head") != -1, joints))
        self.torso_joints = list(filter(lambda joint: joint.lower().find("torso") != -1, joints))

    def _get_raw_meas_qvtau(self):
        """
        Get the current position, velocity and effort of everyjoint of the robot.

        Read it from the ros 'joint_state_topic' topic.

        Optionnals parameters:
        ----------------------
            raw (bool): If not set to True, the configuration will be adjusted to fit in the joints bounds.

        Returns
        -------
            q (float[]): the configuration.
            v (float[]): the velocities.
            tau (float[]): the efforts.

        Raises
        ------
            AssertionError: If the adjusted configuration deviates too much from the original one.
        """
        q = list(pin.neutral(self.pin_robot_wrapper.model))
        v = [0] * self.pin_robot_wrapper.model.nv
        tau = [0] * self.pin_robot_wrapper.model.nv

        msg = self._joint_state_obs.get_last_msg()

        for ros_idx, joint_name in enumerate(msg.name):
            pin_index = self.pin_robot_wrapper.model.names.tolist().index(joint_name)
            pin_nq = self.pin_robot_wrapper.model.joints[pin_index].nq
            if pin_nq != 1:
                continue
            pin_idxq = self.pin_robot_wrapper.model.joints[pin_index].idx_q
            pin_idxv = self.pin_robot_wrapper.model.joints[pin_index].idx_v
            q[pin_idxq] = msg.position[ros_idx]
            v[pin_idxv] = msg.velocity[ros_idx]
            tau[pin_idxv] = msg.effort[ros_idx]

        return q, v, tau

def robot_commanders():
    robot = Tiago_Robot("prl_tiago_description", "/joint_states")
    robot.MAX_JOINT_ACC = 3.1415926 / 1.0 # 180deg/s^2

    commander_left_arm  = Commander(robot, robot.left_arm_joints,  trajectory_action_name="/arm_left_controller",  fwd_action_name="")
    commander_right_arm = Commander(robot, robot.right_arm_joints, trajectory_action_name="/arm_right_controller", fwd_action_name="")
    return robot, commander_left_arm, commander_right_arm