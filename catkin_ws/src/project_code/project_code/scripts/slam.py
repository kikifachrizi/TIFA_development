#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

def pose_callback(data):
    x = data.pose.position.x
    y = data.pose.position.y

    quaternion = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w
    )

    euler = euler_from_quaternion(quaternion)
    theta = euler[2]

    x_cm = x * 100
    y_cm = y * 100 
    theta_deg = theta * (180 / 3.14159265359)

    rospy.loginfo("X: %f, Y: %f, Theta: %f", x, y, theta)
    rospy.loginfo("X : %f cm, Y : %f cm, Theta : %f Derajat", x_cm, y_cm, theta_deg)
    rospy.loginfo("================================================================")
def listener():
    rospy.init_node('hector_slam_listener', anonymous=True)
    rospy.Subscriber("/slam_out_pose", PoseStamped, pose_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
