import rospy
from sensor_msgs.msg import JointState
import numpy as np

rospy.init_node("Viz_debug")
rospy.logwarn("node inited")

from prl_pinocchio.ur5 import robot
rospy.logwarn("Imported robot")

# robot.create_visualizer()
rospy.logwarn("Created viz")


saved = []
def cb(msg):
    saved.append(np.array(msg.position))
    # robot.display(np.array(msg.position))

sub = rospy.Subscriber("joint_states_debug", JointState, cb, queue_size=-1)
rospy.logwarn("Listening")

input("Press enter to finish")
sub.unregister()

import pickle
with open("/tmp/test_bis.pkl", 'wb') as f:
    pickle.dump(saved, f)