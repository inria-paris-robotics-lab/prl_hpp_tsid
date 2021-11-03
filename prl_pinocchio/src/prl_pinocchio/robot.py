import rospy
from sensor_msgs.msg import JointState
from prl_pinocchio.tools.utils import replace_path_to_absolute, compare_configurations
import pinocchio
import numpy


class Robot:
    """ User friendly Robot class that encapsulate the HppRobot and some ROS functionnality """
    MAX_JOINT_SPEED = 1.0

    def __init__(self, robotName, robot_description_param_prefix, joint_state_topic):
        """
        Parameters
        ----------
            robotName (str): see Humanoid Path Planner documentation.
            robot_description_param_prefix (str): Prefix to get ros parameters 'urdf' and 'srdf' (that contains robot urdf and srdf strings).
            joint_state_topic (str): Topic name to get robot configuration.
        """
        urdfString = rospy.get_param(robot_description_param_prefix + "/urdf")
        srdfString = rospy.get_param(robot_description_param_prefix + "/srdf")
        urdfStringExplicit = replace_path_to_absolute(urdfString)
        srdfStringExplicit = replace_path_to_absolute(srdfString)

        self.robotName = robotName
        self.joint_state_topic = joint_state_topic

        self.pin_model = pinocchio.buildModelFromXML(urdfString)
        self.pin_data = self.pin_model.createData()

    def get_meas_q(self, raw=False):
        """
        Get the current configuration of the robot.

        Read it from the ros 'joint_state_topic' topic.

        Optionnals parameters:
        ----------------------
            raw (bool): If not set to True, the configuration will be adjusted to fit in the joints bounds.

        Returns
        -------
            q (float[]): the configuration.

        Raises
        ------
            AssertionError: If the adjusted configuration deviates too much from the original one.
        """
        q, _, _ = self.get_meas_qvtau(raw)
        return q

    def get_meas_qvtau(self, raw=False):
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
        joints_state = rospy.wait_for_message(self.joint_state_topic, JointState)
        q = list(joints_state.position)
        v = list(joints_state.velocity)
        tau = list(joints_state.effort)

        if not raw:
            q_names = self.get_joint_names()
            for i, name in enumerate(q_names):
                bounds = self.hpp_robot.getJointBounds(name)
                bounded = max(bounds[0], min(bounds[1], q[i])) # Bounds the value
                assert abs(bounded - q[i]) < 1e-3, "Joint way out of bounds"
                q[i]=bounded
        
        return q, v, tau

    def get_meas_pose(self, jointName, q=None):
        """
        Get the current 6D pose of a joint.

        Compute the forward kinematic from the configuration.

        Parameters:
        ----------------------
            jointName (str): Name of the joint to read the pose of.

        Optionnals parameters:
        ----------------------
            q (float[]): Configuration of the robot. If None, the current configuration will be read from 'joint_state_topic'.

        Returns
        -------
            xyz_quat (float[3+4]): position and orientation (as a quaternion) coordinates concatenanted.

        Raises
        ------
            AssertionError: If jointName is not in the robot model.
        """
        if(q==None):
            q = self.getCurrentConfig()

        q = numpy.matrix(q).T

        pinocchio.forwardKinematics(self.pin_model, self.pin_data, q)

        frame_index = self.pin_model.getJointId(jointName)
        assert frame_index < len(self.pin_data.oMi), "Joint name not found in robot model : " + jointName

        oMf = self.pin_data.oMi[frame_index]
        xyz_quat = pinocchio.SE3ToXYZQUATtuple(oMf)
        return xyz_quat

    def get_joint_names(self, with_prefix = True):
        """
        Get the name of every 'actuated' joints.

        Optionnals parameters:
        ----------------------
            with_prefix (bool): If set to true, the names will start with '{robotName}/'

        Returns
        -------
            jointNames (str[]): List of the names
        """
        hppJointNames = self.hpp_robot.get_joint_names()

        jointNames = []
        for jointName in hppJointNames:
            if(len(jointName.split(self.robotName + "/")) > 1): # Check that this joint belong to the robot
                if with_prefix:
                    jointNames.append(jointName)
                else:
                    jointNames.append(jointName.split(self.robotName + "/")[1])
        return jointNames

    def is_at_config(self, q, threshold=0.1):
        """
        Check wether the robot is at a certain configuration.

        Compare every joint position to the desired position and check if it is under a certain tolerance.

        Parameters
        ----------
            q (float[]): The desired configuration of the robot.

        Optionnals parameters:
        ----------------------
            threshold (foat): Error tolerance for each joint.

        Returns
        -------
            is_at_config (bool): True if the robot is at the desired configuration.

        Raises
        ------
            AssertionError: If the desired configuration size doesn't match.
        """
        q_curr = self.get_meas_q()
        return compare_configurations(q, q_curr, threshold)
