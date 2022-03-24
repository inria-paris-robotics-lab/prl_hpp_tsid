import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from prl_pinocchio.tools.configurations import ConfigurationConvertor

class Commander:
    """
    This class is in charge of the control of the actual robot.
    """

    DT = 1/125. # control frequency

    def __init__(self, robot, jointsName, trajectory_action_name=None, trajectory_cmd_name=None, fwd_action_name=None, speedScaling = 1.0, accScaling = 1.0):
        """
        Parameters
        ----------
            robot (Robot): Robot class to control. (see robot.py).
            jointsName (str[]): Name of the joints to actually command.
            trajectory_action_name (str): Name of the ros action server for controlling the robot with trajectories.

        Optionnals parameters:
        ----------------------
            speedScaling (float): Ratio to control the execution of path (relative to the robot maximum speed).
        """
        self.jointsName = jointsName
        self.robot = robot

        # Convert configuration from pin to ros
        self.converter = ConfigurationConvertor(self.robot.pin_robot_wrapper.model, self.jointsName)

        self._trajectory_action_name = trajectory_action_name
        self._trajectory_cmd_name = trajectory_cmd_name
        self._fwd_action_name = fwd_action_name

        # Create action client to send the commands
        self._traj_action_client = None
        self._traj_cmd_pub = None
        self._fwd_pub_topic = None

    def start_trajecotry(self):
        """
        Start the commander for trajectory control.

        This requires to have initialized the ROS node already.
        """
        # Check if the action client is already created
        if self._traj_action_client is None:
            if self._trajectory_action_name is not None:
                self._traj_action_client = actionlib.SimpleActionClient(self._trajectory_action_name, FollowJointTrajectoryAction)
                rospy.loginfo("Waiting for action server "  + self._trajectory_action_name)
                self._traj_action_client.wait_for_server(timeout=rospy.Duration.from_sec(10.0))
            else:
                rospy.logwarn("No action server name provided. start_trajecotry() is skipped.")

    def start_trajecotry_cmd(self):
        """
        Start the commander for trajectory control.

        This requires to have initialized the ROS node already.
        """
        # Check if the action client is already created
        if self._traj_cmd_pub is None:
            if self._trajectory_cmd_name is not None:
                self._traj_cmd_pub = rospy.Publisher(self._trajectory_cmd_name, JointTrajectory, queue_size=1)
            else:
                rospy.logwarn("No topic name provided. start_trajecotry_cmd() is skipped.")

    def start_fwd(self):
        """
        Start the commander for fwd control.

        This requires to have initialized the ROS node already.
        """
        # Check if the action client is already created
        if self._fwd_pub_topic is None:
            if self._fwd_action_name is not None:
                self._fwd_pub_topic = rospy.Publisher(self._fwd_action_name + "/command", Float64MultiArray, queue_size=1)
            else:
                rospy.logwarn("No action server name provided. start_fwd() is skipped.")

    def execute_path(self, path, wait=True):
        """
        Execute a path on the robot.

        Parameters
        ----------
            path (Path): path to execute.

        Raises
        ------
            AssertionError: If the start configuration of the path differs too much from the actual robot configuration.
            AssertionError: If the one or more commanded joint from this Commander is not in the path joints.
            AssertionError: If the action client is not initialized.
        """
        if self._traj_action_client is None:
            rospy.logerr("Action client not initialized. Did you call start_trajecotry() first.")
            raise AssertionError("Action client not initialized. Did you call start_trajecotry() first.")

        for i in range(len(path.jointList)):
            if(self.robot.pin_robot_wrapper.model.names[i+1] != path.jointList[i]):
                print(self.robot.pin_robot_wrapper.model.names[i+1], path.jointList[i])

        # Assume that the robot is always the first components of the configuraitons
        def q_hpp_to_pin(q):
            return q[:self.robot.pin_robot_wrapper.model.nq]
        def v_hpp_to_pin(v):
            return v[:self.robot.pin_robot_wrapper.model.nv]

        # Check that the robot is close to the start configuration
        q_start = q_hpp_to_pin(path.corbaPath.call(0)[0])
        assert self.robot.is_at_config(q_start, 1e-1), "The robot current configuration differs too much from the start configuration of the path"

        # Create ROS message
        jointTraj = JointTrajectory(joint_names = self.jointsName)

        # Make point array
        t = 0
        while t < path.corbaPath.length():
            t_ros = rospy.Time.from_sec(t)
            q = self.converter.q_pin_to_ros(q_hpp_to_pin(path.corbaPath.call(t)[0]))
            q_dot = self.converter.v_pin_to_ros(v_hpp_to_pin(path.corbaPath.derivative(t, 1)))
            point = JointTrajectoryPoint(positions = q, velocities = q_dot, time_from_start = t_ros)
            jointTraj.points.append(point)
            t += self.DT

        # Send trajectory to controller
        jointTraj.header.stamp = rospy.Time(0)
        self._traj_action_client.send_goal(FollowJointTrajectoryGoal(trajectory=jointTraj))

        # Wait for the path to be fully executed
        if wait:
            self._traj_action_client.wait_for_result()

    def execute_step_trajecotry(self, q_curr, v_curr, q_next, v_next, dv, dt):
        """
        Execute a (q, v, dv) for a dt period on the robot.

        Parameters
        ----------
            q_next  (float[]): joint position reference.
            v_next  (float[]): joint velocity reference.
            dv      (float[]): joint acceleration reference.
            dt      (float[]): Duration during which to apply dt in order to get to (v_next, q_next).
        """
        if self._traj_cmd_pub is None:
            rospy.logerr("Action client not initialized. Did you call start_trajecotry_cmd() first.")
            raise AssertionError("Action client not initialized. Did you call start_trajecotry_cmd() first.")

        # q_curr = self.converter.q_pin_to_ros(q_curr)
        # v_curr = self.converter.v_pin_to_ros(v_curr)

        # Filter joints
        q_next = self.converter.q_pin_to_ros(q_next)
        v_next = self.converter.v_pin_to_ros(v_next)
        dv = self.converter.v_pin_to_ros(dv)

        # Create ROS message
        jointTraj = JointTrajectory(joint_names = self.jointsName, points = [ # JointTrajectoryPoint(positions = q_curr, velocities = v_curr, accelerations = dv, time_from_start = rospy.Time.from_sec(0)),
                                                                              JointTrajectoryPoint(positions = q_next, velocities = v_next, accelerations = dv, time_from_start = rospy.Time.from_sec(dt)),])

        # Make header timestamp and send goal to controller
        jointTraj.header.stamp = rospy.Time(0)
        self._traj_cmd_pub.publish(jointTraj)

    def execute_step_fwd(self, v):
        """
        Execute a velocity on the robot.

        Parameters
        ----------
            v (float[]): joint velocity reference.

        Raises
        ------
            AssertionError: If the action client is not initialized.
        """
        if self._fwd_pub_topic is None:
            rospy.logerr("Action client not initialized. Did you call start_fwd() first.")
            raise AssertionError("Action client not initialized. Did you call start_fwd() first.")

        # Filter joints
        v = [v[i] for i in self._commanded_joints_indexes]

        # Create ROS message
        fwd = Float64MultiArray()
        fwd.data = v
        fwd.layout.dim.append(MultiArrayDimension(label='nv', size=len(v)))

        # Send goal to controller
        self._fwd_pub_topic.publish(fwd)