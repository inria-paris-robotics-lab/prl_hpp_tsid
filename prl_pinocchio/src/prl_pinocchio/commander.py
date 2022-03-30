import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from prl_pinocchio.tools.configurations import ConfigurationConvertor

class Commander:
    """
    This class is in charge of the control of the actual robot.
    """

    def __init__(self, robot, jointsName, trajectory_action_name=None, fwd_topic_name=None, speedScaling = 1.0, accScaling = 1.0):
        """
        Parameters
        ----------
            robot (Robot): Robot class to control. (see robot.py).
            jointsName (str[]): Name of the joints to actually command.
            trajectory_action_name (str): Name of the ros action server for controlling the robot with trajectories.
            fwd_topic_name (str): Name of the ros topic for controlling the robot directly.

        Optionnals parameters:
        ----------------------
            speedScaling (float): Ratio to control the execution of path (relative to the robot maximum speed).
            accScaling (float): Ratio to control the execution of path (relative to the robot maximum acceleration).
        """
        self.jointsName = jointsName
        self.robot = robot

        # Convert configuration from pin to ros
        self.converter = ConfigurationConvertor(self.robot.pin_robot_wrapper.model, self.jointsName)

        self._trajectory_action_name = trajectory_action_name
        self._fwd_topic_name = fwd_topic_name

        # Create action client to send the commands
        self._traj_action_client = None
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

    def start_fwd(self):
        """
        Start the commander for fwd control.

        This requires to have initialized the ROS node already.
        """
        from joint_group_ff_controllers.msg import setpoint

        # Check if the action client is already created
        if self._fwd_pub_topic is None:
            if self._fwd_topic_name is not None:
                self._fwd_pub_topic = rospy.Publisher(self._fwd_topic_name, setpoint, queue_size=1)
            else:
                rospy.logwarn("No action server name provided. start_fwd() is skipped.")

    def execute_path(self, path, dt=1/125., wait=True):
        """
        Execute a path on the robot.

        Parameters
        ----------
            path (Path): path to execute.

        Optionnal parameters:
        ---------------------
            dt (float): Time step to discretize the path.
            wait (bool): If True, wait for the execution of the path to finish before returning.

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

    def execute_fwd(self, q, v, tau, timeout):
        """
        Execute a velocity on the robot.

        Parameters
        ----------
            q (float[]): Goal configuration of the robot.
            v (float[]): Goal velocity of the robot.
            tau (float[]): Goal torque of the robot.
            timeout (float): Timeout for the execution of the commands.

        Raises
        ------
            AssertionError: If the action client is not initialized.
        """
        if self._fwd_pub_topic is None:
            rospy.logerr("Action client not initialized. Did you call start_fwd() first.")
            raise AssertionError("Action client not initialized. Did you call start_fwd() first.")

        # Filter joints
        q = self.converter.q_pin_to_ros(q)
        v = self.converter.v_pin_to_ros(v)
        tau = self.converter.v_pin_to_ros(tau)

        # Send goal to controller
        self._fwd_pub_topic.publish(positions = q, velocities = v, efforts = tau, timeout = rospy.Duration(timeout))