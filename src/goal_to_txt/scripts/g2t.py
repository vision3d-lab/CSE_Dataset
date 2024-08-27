#!/usr/bin/env python
import rospy
# from std_msgs.msg import 
from geometry_msgs.msg import PoseStamped

class Listener:
    def __init__(self):
        rospy.init_node('g2t', anonymous=True)
        rospy.Subscriber("carter_msg_goal", PoseStamped, self.callback)
        self.file = open("goals.txt", "w")

    def callback(self, data):
        message_str = "{} {} {} {} {} {}\n".format(
            data.pose.position.x, data.pose.position.y,
            data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
        rospy.loginfo(rospy.get_caller_id() + "I heard: \n" + message_str)
        self.file.write(message_str)
        
    def listener_spin(self):
        rospy.spin()
        self.file.close()

if __name__ == '__main__':
    l = Listener()
    l.listener_spin()
