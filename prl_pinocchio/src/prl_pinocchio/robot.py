import rospy
from sensor_msgs.msg import JointState
from prl_pinocchio.tools.utils import replace_path_to_absolute, compare_configurations
import pinocchio
import numpy


class Robot:
    """ User friendly Robot class that encapsulate the HppRobot and some ROS functionnality """
    MAX_JOINT_SPEED = 1.0

    def __init__(self, robot_description_param_prefix, joint_state_topic):
        """
        Parameters
        ----------
            robot_description_param_prefix (str): Prefix to get ros parameters 'urdf' and 'srdf' (that contains robot urdf and srdf strings).
            joint_state_topic (str): Topic name to get robot configuration.
        """
        # Get URDF/SRDF strings from ros parameters
        urdfString = rospy.get_param(robot_description_param_prefix + "/urdf")
        srdfString = rospy.get_param(robot_description_param_prefix + "/srdf")
        self._urdfStringExplicit = replace_path_to_absolute(urdfString)
        self._srdfStringExplicit = replace_path_to_absolute(srdfString)

        # Joint topic from ros
        self.joint_state_topic = joint_state_topic

        # Build pinocchio model
        self.pin_model = pinocchio.buildModelFromXML(urdfString)
        self.pin_data = self.pin_model.createData()

        # # Check that for each ros joint the corresponding pinocchio model joint matches the name and index
        # ros_joint_names = rospy.wait_for_message(self.joint_state_topic, JointState).name
        # assert len(ros_joint_names) == len(self.pin_model.names[1:]), \
        #        F"Error number of joints differ between ROS and pinocchio model : {len(ros_joint_names)} != {len(self.pin_model.names[1:])}"
        #        # Pinocchio joints starts at 1 to take into account the universe joint that is not in ros.
        # for index, name in enumerate(ros_joint_names):
        #     assert self.pin_model.names[index+1] == name, \
        #            F"Error while matching ROS and pinocchio model joints at index {index} : joint name {self.pin_model.names[index+1]} differs from {name}."
        #            # Pinocchio index starts at +1 to take into account the universe joint.

        # Make a lookup table to change between ros joint index and pinocchio joint index (using joints names)
        # The input/output of this class will always be according to pinocchio joint order
        ros_joint_names = list(rospy.wait_for_message(self.joint_state_topic, JointState).name)
        pin_joint_names = list(self.pin_model.names[1:]) # Pinocchio joints starts at 1 to take into account the universe joint that is not in ros.

        self._lookup_ros_to_pin = []
        for name in ros_joint_names:
            pin_index = pin_joint_names.index(name)
            self._lookup_ros_to_pin.append(pin_index)

        self._lookup_pin_to_ros = []
        for name in pin_joint_names:
            pin_index = ros_joint_names.index(name)
            self._lookup_pin_to_ros.append(pin_index)

    def _rearrange_ros_to_pin(self, q=None, v=None, tau=None):
        # TODO: If joints makes nq != nv what to do ?!!
        res  = []

        if q != None:
            q_res = [q[self._lookup_pin_to_ros[i]] for i in range(len(q))]
            res.append(q_res)
        
        if v != None:
            v_res = [v[self._lookup_pin_to_ros[i]] for i in range(len(v))]
            res.append(v_res)

        if tau != None:
            tau_res = [tau[self._lookup_pin_to_ros[i]] for i in range(len(tau))]
            res.append(tau_res)

        return res

    def get_urdf_explicit(self):
        return self._urdfStringExplicit

    def get_srdf_explicit(self):
        return self._srdfStringExplicit

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

        q, v, tau = self._rearrange_ros_to_pin(q=q, v=v, tau=tau)

        if not raw:
            for i, pos in enumerate(q):
                pos = max(pos, self.pin_model.lowerPositionLimit[i])
                pos = min(pos, self.pin_model.upperPositionLimit[i])
                assert abs(pos - q[i]) < 1e-3, F"Joint {i} way out of bounds : {self.pin_model.names[i+1]}"
                q[i] = pos
        
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

    def get_joint_names(self):
        """
        Get the name of every 'actuated' joints.

        Returns
        -------
            jointNames (str[]): List of the names
        """
        return list(self.pin_model.names[1:])

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
