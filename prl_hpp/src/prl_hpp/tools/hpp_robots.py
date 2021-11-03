from hpp.corbaserver.manipulation.robot import Robot as Parent
from prl_hpp.tools.utils import replace_path_to_absolute, replace_placeholders

class HppRobot(Parent):
    """Robot in term of what HPP need for its planning """

    def __init__(self, compositeName, robotName, urdfString, srdfString, rootJointType = "anchor"):
        """
        Parameters
        ----------
            see Humanoid Path Planner documentation.
        """
        self.robotName = robotName
        self.urdfString = urdfString
        self.srdfString = srdfString
        Parent.__init__ (self, compositeName, robotName, rootJointType, load = True)

    def get_joint_names(self):
        """
        Get the names of every 'actuated' joints.

        Returns
        -------
            jointNames (str[]): Names of the joints
        """
        actuatedJoints = []
        for jointName in self.getAllJointNames():
            if(self.getJointNumberDof(jointName)):
                actuatedJoints.append(jointName)
        return actuatedJoints

class TargetRobotStrings:
    """Prepare and holds the urdf and srdf strings of the Target Robot (i.e. a robot that is just a freeflyer with a handle, and no collisions)"""

    _urdfFilename = replace_path_to_absolute("package://prl_hpp/" + "urdf_srdf/dummy.urdf")
    _srdfFilename = replace_path_to_absolute("package://prl_hpp/" + "urdf_srdf/dummy.srdf")

    def __init__(self, clearance):
        # Read file in strings
        urdf_file = open(self._urdfFilename, mode='r')
        srdf_file = open(self._srdfFilename, mode='r')

        self.urdf = urdf_file.read()
        self.srdf = srdf_file.read()

        urdf_file.close()
        srdf_file.close()

        # Replace clearance placeHolder
        self.srdf = replace_placeholders(self.srdf, "{CLEARANCE}", str(clearance))
