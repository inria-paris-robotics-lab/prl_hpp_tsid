# Do this first as prl_hpp.ur5 requires the node to be already started
import rospy
rospy.init_node("TSID", anonymous=True)

import tsid
from prl_pinocchio.ur5 import robot, commander_left_arm, commander_right_arm

import numpy as np

# TSID Robot
tsid_robot = tsid.RobotWrapper(robot.pin_robot_wrapper.model, True, False)

# Base configurations
q0 = np.zeros(tsid_robot.nq)
v0 = np.zeros(tsid_robot.nv)
a0 = np.zeros(tsid_robot.na)

# Robot displays
robot.create_visualizer()
robot.display(q0)

# Forumlation
formulation = tsid.InverseDynamicsFormulationAccForce("tsid", tsid_robot, False)
formulation.computeProblemData(0.0, q0, v0)