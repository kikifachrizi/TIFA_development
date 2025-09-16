#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('test_node')
    pub = rospy.Publisher("chatter", String, queue_size=10)
    r = rospy.Rate(10)
    rospy.loginfo("Heloooo bangg")
    rospy.logwarn("Warning bangg")
    rospy.logerr("Error bangg")
    counter = 0
    while not rospy.is_shutdown():
        hellomsg = "helllo %d" %counter
        pub.publish(hellomsg)
        rospy.sleep(1)
        counter += 1

    

    rospy.loginfo("Exitttt")