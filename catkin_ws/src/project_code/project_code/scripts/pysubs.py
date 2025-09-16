#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def msgcallbck(msg):
    rospy.loginfo("New Massage received %s", msg.data)

if __name__ == '__main__':
    rospy.init_node('py_subs', anonymous=True)
    rospy.Subscriber("chatter", String, msgcallbck)

    rospy.spin()
    r = rospy.Rate(10)
    rospy.loginfo("Heloooo bangg")
    rospy.logwarn("Warning bangg")
    rospy.logerr("Error bangg")
    counter = 0

    

    rospy.loginfo("Exitttt")