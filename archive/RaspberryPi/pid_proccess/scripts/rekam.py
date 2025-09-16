#!/usr/bin/env python3

import rospy
import csv
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

def pose_callback(msg):
    x = msg.pose.position.x
    y = msg.pose.position.y
    # th = msg.pose.orientation.z
    orientation_q = msg.pose.orientation
    
    (roll, pitch, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    yaw_deg = yaw * 180.0 / 3.141592653589793

    print("x: ", x)
    print("y: ", y)
    print("th: ", yaw_deg)
    with open('trajectoryNew.csv', mode='a') as file:
        writer = csv.writer(file)
        writer.writerow([x, y, yaw_deg])

def listener():

    rospy.init_node('slam_to_csv', anonymous=True)
    
    with open('trajectory5.csv', mode='w') as file:
        writer = csv.writer(file)
        writer.writerow(['x', 'y', 'th']) 

    rospy.Subscriber('/slam_out_pose', PoseStamped, pose_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
