import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class Commander:
    """
    This class is in charge of the control of the actual robot.
    """

    def __init__(self, robot, commandedJointsName, action_name):
        """
        Parameters
        ----------
            robot (Robot): Robot class to control. (see robot.py).
            jointsName (str): Name of the joints to actually command.
            action_name (str): Name of the ros action server for controlling the robot.
        """
        self.commandedJointsName = commandedJointsName
        self.robot = robot
        self.commandedJointsIndex = self._getJointIndexes(commandedJointsName, self.robot.get_joint_names())

        self.action_client = actionlib.SimpleActionClient(action_name, FollowJointTrajectoryAction)
        self.action_client.wait_for_server()

    def execute(self, q_next, v_next, dv, dt):
        """
        Execute a (q, v, dv) on the robot.

        Parameters
        ----------
            q_next  (float[]): joint position reference.
            v_next  (float[]): joint velocity reference.
            dv      (float[]): joint acceleration reference.
            dt      (float[]): Duration during which to apply dt in order to get to (v_next, q_next).
        """
        # Filter joints
        q_next = [q_next[i] for i in self.commandedJointsIndex]
        v_next = [v_next[i] for i in self.commandedJointsIndex]
        dv = [dv[i] for i in self.commandedJointsIndex]

        # Create ROS message
        jointTraj = JointTrajectory(joint_names = self.commandedJointsName, points = [JointTrajectoryPoint(positions = q_next, velocities = v_next, accelerations = dv, time_from_start = rospy.Time.from_sec(dt)),])

        # Make header timestamp and send goal to controller
        jointTraj.header.stamp = rospy.get_rostime()
        self.action_client.send_goal(FollowJointTrajectoryGoal(trajectory=jointTraj))

    def _getJointIndexes(self, jointSubSet, jointSet, strict = True):
        indexes = []
        for sub_joint in jointSubSet:
            for i, joint in enumerate(jointSet):
                if sub_joint == joint:
                    indexes.append(i)
                    break #next joint
        assert (not strict) or (len(indexes)==len(jointSubSet)), "Not all joint from the subset could be found in the set for :" + str(jointSubSet) + "\n in :" + str(jointSet)
        return indexes
