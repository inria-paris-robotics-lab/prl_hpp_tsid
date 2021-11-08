import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class Commander:
    """
    This class is in charge of the control of the actual robot.
    """

    DT = 1/125. # control frequency

    def __init__(self, robot, jointsName, trajectory_action_name, speedScaling = 1.0, accScaling = 1.0):
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

        # Create action client to send the commands 
        self._action_client = actionlib.SimpleActionClient(trajectory_action_name, FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for action server "  + trajectory_action_name)
        self._action_client.wait_for_server()

        # Extract indexes of the joints from the full robot to actually command with this commander
        self._commanded_joints_indexes = self._get_joint_indexes(jointsName, self.robot.get_joint_names())

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
        """
        # Find indexes of the commanded joints in the path
        commanded_joints_indexes = self._get_joint_indexes(self.jointsName, path.jointList)
        robot_joints_indexes = self._get_joint_indexes(self.robot.get_joint_names(), path.jointList)

        # Small function to extract only interresting joints from a configuration in path
        def filterJoints(q_in, joints):
            q_out = [q_in[index] for index in joints]
            return q_out

        # Check that the robot is close to the start configuration
        q_start = filterJoints(path.corbaPath.call(0)[0], robot_joints_indexes)
        assert self.robot.is_at_config(q_start), "The robot current configuration differs too much from the start configuration of the path"

        # Create ROS message
        jointTraj = JointTrajectory(joint_names = self.jointsName)

        # Make point array
        t = 0
        while t < path.corbaPath.length():
            t_ros = rospy.Time.from_sec(t)
            q = filterJoints(path.corbaPath.call(t)[0], commanded_joints_indexes)
            q_dot = filterJoints(path.corbaPath.derivative(t, 1), commanded_joints_indexes)
            point = JointTrajectoryPoint(positions = q, velocities = q_dot, time_from_start = t_ros)
            jointTraj.points.append(point)
            t += self.DT

        # Make header timestamp and send goal to controller
        jointTraj.header.stamp = rospy.get_rostime()
        self._action_client.send_goal(FollowJointTrajectoryGoal(trajectory=jointTraj))

        # Wait for the path to be fully executed
        if wait:
            self._action_client.wait_for_result()

    def execute_step(self, q_next, v_next, dv, dt):
        """
        Execute a (q, v, dv) for a dt period on the robot.

        Parameters
        ----------
            q_next  (float[]): joint position reference.
            v_next  (float[]): joint velocity reference.
            dv      (float[]): joint acceleration reference.
            dt      (float[]): Duration during which to apply dt in order to get to (v_next, q_next).
        """
        # Filter joints
        q_next = [q_next[i] for i in self._commanded_joints_indexes]
        v_next = [v_next[i] for i in self._commanded_joints_indexes]
        dv = [dv[i] for i in self._commanded_joints_indexes]

        # Create ROS message
        jointTraj = JointTrajectory(joint_names = self.jointsName, points = [JointTrajectoryPoint(positions = q_next, velocities = v_next, accelerations = dv, time_from_start = rospy.Time.from_sec(dt)),])

        # Make header timestamp and send goal to controller
        jointTraj.header.stamp = rospy.get_rostime()
        self._action_client.send_goal(FollowJointTrajectoryGoal(trajectory=jointTraj))

    def _get_joint_indexes(self, jointSubSet, jointSet, strict = True):
        indexes = []
        for sub_joint in jointSubSet:
            for i, joint in enumerate(jointSet):
                if sub_joint == joint:
                    indexes.append(i)
                    break #next joint
        assert (not strict) or (len(indexes)==len(jointSubSet)), "Not all joint from the subset could be found in the set for :" + str(jointSubSet) + "\n in :" + str(jointSet)
        return indexes
