import rospy
from prl_pinocchio.tools.utils import replace_path_to_absolute, compare_configurations
import pinocchio
import numpy


class Robot:
    """ User friendly Robot class that encapsulate the HppRobot and some ROS functionnality """
    def __init__(self, robot_description_param_prefix):
        """
        Parameters
        ----------
            robot_description_param_prefix (str): Prefix to get ros parameters 'urdf' and 'srdf' (that contains robot urdf and srdf strings).
        """
        # Get URDF/SRDF strings from ros parameters
        urdfString = rospy.get_param(robot_description_param_prefix + "/urdf")
        srdfString = rospy.get_param(robot_description_param_prefix + "/srdf")
        self._urdfStringExplicit = replace_path_to_absolute(urdfString)
        self._srdfStringExplicit = replace_path_to_absolute(srdfString)

        # Build pinocchio models and wrappers
        pin_model = pinocchio.buildModelFromXML(urdfString)
        pin_collision_model = pinocchio.buildGeomFromUrdfString(pin_model, self.get_urdf_explicit(), pinocchio.COLLISION)
        pin_visual_model    = pinocchio.buildGeomFromUrdfString(pin_model, self.get_urdf_explicit(), pinocchio.VISUAL)

        self.pin_robot_wrapper = pinocchio.RobotWrapper(pin_model, pin_collision_model, pin_visual_model)

    def get_urdf_explicit(self):
        return self._urdfStringExplicit

    def get_srdf_explicit(self):
        return self._srdfStringExplicit

    def get_meas_q(self, raw=False):
        """
        Get the current configuration of the robot.

        Read it from the ros topics (e.g /joint_states).

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

        Read it from the ros topics (e.g /joint_states).

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
        q, v, tau = self._get_raw_meas_qvtau()

        if not raw:
            for i, pos in enumerate(q):
                pos = max(pos, self.pin_robot_wrapper.model.lowerPositionLimit[i])
                pos = min(pos, self.pin_robot_wrapper.model.upperPositionLimit[i])
                assert abs(pos - q[i]) < 1e-3 #, F"Joint {i} way out of bounds : {self.pin_robot_wrapper.pin_model.names[i+1]}"
                q[i] = pos

        return q, v, tau

    def _get_raw_meas_qvtau(self):
        """
        Get the current position, velocity and effort of everyjoint of the robot and output the configuration in pinocchio joint order (as opposed to ros order that could be different).

        Read it from the ros topics (e.g /joint_states).

        Returns
        -------
            q (float[]): the configuration (pinocchio joint order).
            v (float[]): the velocities (pinocchio joint order).
            tau (float[]): the efforts (pinocchio joint order).

        Raises
        ------
            AssertionError: If the adjusted configuration deviates too much from the original one.
        """
        raise NotImplementedError

    def get_joint_pose(self, jointName, q=None):
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
            q = self.get_meas_q()

        q = numpy.matrix(q).T

        frame_index = self.pin_robot_wrapper.model.getJointId(jointName)
        assert frame_index < len(self.pin_robot_wrapper.data.oMi), "Joint name not found in robot model : " + jointName

        oMi = self.pin_robot_wrapper.placement(q, frame_index)
        xyz_quat = pinocchio.SE3ToXYZQUATtuple(oMi)
        return xyz_quat

    def get_frame_pose(self, frameName, q=None):
        """
        Get the current 6D pose of a frame.

        Compute the forward kinematic from the configuration.

        Parameters:
        ----------------------
            frameName (str): Name of the frame to read the pose of.

        Optionnals parameters:
        ----------------------
            q (float[]): Configuration of the robot. If None, the current configuration will be read from 'joint_state_topic'.

        Returns
        -------
            xyz_quat (float[3+4]): position and orientation (as a quaternion) coordinates concatenanted.

        Raises
        ------
            AssertionError: If frameName is not in the robot model.
        """
        if(q==None):
            q = self.get_meas_q()

        q = numpy.matrix(q).T

        frame_index = self.pin_robot_wrapper.model.getFrameId(frameName)
        assert frame_index < len(self.pin_robot_wrapper.data.oMf), "Frame name not found in robot model : " + frameName

        oMf = self.pin_robot_wrapper.framePlacement(q, frame_index)
        xyz_quat = pinocchio.SE3ToXYZQUATtuple(oMf)
        return xyz_quat

    def get_joint_names(self):
        """
        Get the name of every 'actuated' joints as it's defined in pinocchio from the urdf model.

        Returns
        -------
            jointNames (str[]): List of the names
        """
        return list(self.pin_robot_wrapper.model.names[1:])

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
        return compare_configurations(self.pin_robot_wrapper.model, q, q_curr, threshold)

    def create_visualizer(self):
        # from pinocchio.visualize import RVizVisualizer
        # self.pin_robot_wrapper.setVisualizer(RVizVisualizer())
        # self.pin_robot_wrapper.initViewer(loadModel=True, initRosNode=False)
        from pinocchio.visualize import GepettoVisualizer
        self.pin_robot_wrapper.setVisualizer(GepettoVisualizer())
        self.pin_robot_wrapper.initViewer(loadModel=True)

    def display(self, q):
        self.pin_robot_wrapper.display(q)

    def create_dof_lookup(self, new_joint_list):
        pin_model = self.pin_robot_wrapper.model
        pin_joint_names = self.get_joint_names()

        # enumerate all the DoF and group them by joint
        index = 0
        q_pin_to_pin = []
        for joint in pin_model.joints:
            joint_indexes = []
            for i in range(joint.nq):
                joint_indexes.append(index)
                index+=1
            q_pin_to_pin.append(joint_indexes)
        # rearrange joints
        q_new_to_pin = []
        for new_index, name in enumerate(new_joint_list):
            pin_index = pin_joint_names.index(name)
            q_new_to_pin.append(q_pin_to_pin[pin_index])
        # Flatten the list of list into a simple list
        res_q_new_to_pin = []
        for indexes in q_new_to_pin:
            res_q_new_to_pin.extend(indexes)
        # Take the inverse of the bijection
        res_q_pin_to_new = []
        for pin_index in range(len(res_q_new_to_pin)):
            new_index = res_q_new_to_pin.index(pin_index)
            res_q_pin_to_new.append(new_index)

        # re-do all of the previous for v (instead of q)
        # enumerate all the DoF and group them by joint
        index = 0
        v_pin_to_pin = []
        for joint in pin_model.joints:
            joint_indexes = []
            for i in range(joint.nv):
                joint_indexes.append(index)
                index+=1
            v_pin_to_pin.append(joint_indexes)
        # rearrange joints
        v_new_to_pin = []
        for new_index, name in enumerate(new_joint_list):
            pin_index = pin_joint_names.index(name)
            v_new_to_pin.append(v_pin_to_pin[pin_index])
        # Flatten the list of list into a simple list
        res_v_new_to_pin = []
        for indexes in v_new_to_pin:
            res_v_new_to_pin.extend(indexes)
        # Take the inverse of the bijection
        res_v_pin_to_new = []
        for pin_index in range(len(res_v_new_to_pin)):
            new_index = res_v_new_to_pin.index(pin_index)
            res_v_pin_to_new.append(new_index)

        return res_q_pin_to_new, res_q_new_to_pin, res_v_pin_to_new, res_v_new_to_pin