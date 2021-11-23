import rospy

class Observer:

    def __init__(self, topic, msg_type):
        self.topic = topic
        self.msg_type = msg_type

        self.sub = rospy.Subscriber(topic, msg_type, self._callback)
        self.last_msg = None

    def _callback(self, msg):
        self.last_msg = msg

    def get_last_msg(self, wait_for_first=True):
        if wait_for_first:
            while self.last_msg is None:
                rospy.sleep(0.01)
        return self.last_msg