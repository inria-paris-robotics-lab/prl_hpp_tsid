import rospy
from sensor_msgs.msg import JointState
from prl_pinocchio.tools.observer import Observer
from prl_pinocchio.tools.configurations import ConfigurationConvertor
from prl_pinocchio.robot import Robot
from prl_pinocchio.commander import Commander
"""
 Create instances of Robot dedicated for the Double-UR5 robot.
"""

class UR5_Robot(Robot):
    left_gripper_name = "l_gripper"
    right_gripper_name = "r_gripper"

    def __init__(self, robot_description_param_prefix, joint_state_topic):
        """
        Parameters
        ----------
            robot_description_param_prefix (str): Prefix to get ros parameters 'urdf' and 'srdf' (that contains robot urdf and srdf strings).
            joint_state_topic (str): Topic name to get robot configuration.
        """
        Robot.__init__(self, robot_description_param_prefix)

        # Init joint reading
        # Joint topic from ros
        self.joint_state_topic = joint_state_topic

        self._joint_state_obs = Observer(joint_state_topic, JointState)

        # Prepare lookup table to re-arrange q, v, a from ros to pinocchio format, etc..
        self._configuration_convertor = ConfigurationConvertor(self.pin_robot_wrapper.model, self._joint_state_obs.get_last_msg().name)

        # Init joints group
        joints = self.get_joint_names()
        self.gripper_joints = list(filter(lambda joint: joint.lower().find("gripper") != -1, joints))

        # joints = set(joints) - set(self.gripper_joints) # Remove grippers joints from list
        self.left_arm_joints  = list(filter(lambda joint: joint.lower().find("left")  != -1 and joint.lower().find("gripper") == -1, joints))
        self.right_arm_joints = list(filter(lambda joint: joint.lower().find("right") != -1 and joint.lower().find("gripper") == -1, joints))

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
        joints_state = self._joint_state_obs.get_last_msg()

        return  joints_state.header.stamp.to_sec(), \
                self._configuration_convertor.q_ros_to_pin(joints_state.position), \
                self._configuration_convertor.v_ros_to_pin(joints_state.velocity), \
                self._configuration_convertor.v_ros_to_pin(joints_state.effort)

robot = UR5_Robot("prl_ur5_description", "joint_states")

# Arbitrary value (as velocity and effort limits are already defined in the model)
robot.MAX_JOINT_ACC = 3.1415926 / 1.0 # 180°/s^2

commander_left_arm = Commander(robot, robot.left_arm_joints, trajectory_action_name="/left_arm/scaled_vel_joint_traj_controller/follow_joint_trajectory", fwd_topic_name="/left_arm/ff_controller/command")
commander_right_arm = Commander(robot, robot.right_arm_joints, trajectory_action_name="/right_arm/scaled_vel_joint_traj_controller/follow_joint_trajectory", fwd_topic_name="/right_arm/ff_controller/command")