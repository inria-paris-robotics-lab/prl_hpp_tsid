import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class Commander:
    """
    This class is in charge of the control of the actual robot.
    """

    DT = 1/125. # control frequency

    def __init__(self, robot, jointsName, action_name, speedScaling = 1.0, accScaling = 1.0):
        """
        Parameters
        ----------
            robot (Robot): Robot class to control. (see robot.py).
            jointsName (str): Name of the joints to actually command.
            action_name (str): Name of the ros action server for controlling the robot.

        Optionnals parameters:
        ----------------------
            speedScaling (float): Ratio to control the execution of path (relative to the robot maximum speed).
        """
        self.jointsName = jointsName
        self.robot = robot
        self.action_client = actionlib.SimpleActionClient(action_name, FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for action server "  + action_name)
        self.action_client.wait_for_server()

    def execute(self, path):
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
        commandJoints = self._get_joint_indexes(self.jointsName, path.jointList)
        robotJoints = self._get_joint_indexes(self.robot.get_joint_names(), path.jointList)

        def filterJoints(q_in, joints = commandJoints): # Small function to extract only interresting joints from a configuration in path
            q_out = [q_in[index] for index in joints]
            return q_out

        q_start = filterJoints(path.corbaPath.call(0)[0], robotJoints)
        assert self.robot.is_at_config(q_start), "The robot current configuration differs too much from the start configuration of the path"
        # TODO : also check that the path is still valid with no collisions ?

        # Create ROS message
        jointTraj = JointTrajectory(joint_names = self.jointsName)

        # Make point array
        t = 0
        while t < path.corbaPath.length():
            t_ros = rospy.Time.from_sec(t)
            q = filterJoints(path.corbaPath.call(t)[0])
            q_dot = filterJoints(path.corbaPath.derivative(t, 1))
            point = JointTrajectoryPoint(positions = q, velocities = q_dot, time_from_start = t_ros)
            jointTraj.points.append(point)
            t += self.DT

        # Make header timestamp and send goal to controller
        jointTraj.header.stamp = rospy.get_rostime()
        self.action_client.send_goal(FollowJointTrajectoryGoal(trajectory=jointTraj))
        self.action_client.wait_for_result()
        # print(self.action_client.get_result())

    def _get_joint_indexes(self, jointSubSet, jointSet, strict = True):
        indexes = []
        for sub_joint in jointSubSet:
            for i, joint in enumerate(jointSet):
                if sub_joint == joint:
                    indexes.append(i)
                    break #next joint
        assert (not strict) or (len(indexes)==len(jointSubSet)), "Not all joint from the subset could be found in the set for :" + str(jointSubSet) + "\n in :" + str(jointSet)
        return indexes
