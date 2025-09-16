#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def odom_callback(data):
    rospy.loginfo("Position: x=%.2f, y=%.2f, z=%.2f", 
                  data.pose.pose.position.x, 
                  data.pose.pose.position.y, 
                  data.pose.pose.position.z)

def main():
    rospy.init_node('odom_listener', anonymous=True)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
