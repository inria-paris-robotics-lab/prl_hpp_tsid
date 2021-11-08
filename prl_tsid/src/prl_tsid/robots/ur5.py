# Do this first as prl_hpp.ur5 requires the node to be already started
import rospy
rospy.init_node("TSID", anonymous=True)

import tsid
from prl_pinocchio.ur5 import robot, commander_left_arm, commander_right_arm

import numpy as np

tsid_robot = tsid.RobotWrapper(robot.pin_robot_wrapper.model, True, False)

q0 = np.zeros(tsid_robot.nq)
v0 = np.zeros(tsid_robot.nv)
a0 = np.zeros(tsid_robot.na)

robot.create_visualizer()
robot.display(q0)

formulation = tsid.InverseDynamicsFormulationAccForce("tsid", tsid_robot, False)
formulation.computeProblemData(0.0, q0, v0)