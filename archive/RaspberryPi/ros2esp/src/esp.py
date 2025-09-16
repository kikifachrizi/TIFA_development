#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

def talker():
    pub = rospy.Publisher('position_orientation', Int32, queue_size=10)
    # rospy.init_node('talker_node', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        value = [x, y, theta]
        array_msg = Int32MultiArray()
        array_msg.data = value
        rospy.loginfo(f"Sending: {value}")
        pub.publish(array_msg)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

