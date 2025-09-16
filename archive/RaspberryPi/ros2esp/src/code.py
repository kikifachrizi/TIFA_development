#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

pub = rospy.Publisher('/position_orientation', Int32MultiArray, queue_size=10)

def talker(x, y, theta):
    value = [x, y, theta]
    array_msg = Int32MultiArray()
    array_msg.data = value
    rospy.loginfo(f"Sending: {value}")
    pub.publish(array_msg)

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
    theta_deg = theta * (180 / math.pi)

    rospy.loginfo("X: %f, Y: %f, Theta: %f", x, y, theta)
    rospy.loginfo("====================================")
    rospy.loginfo("X : %f cm, Y : %f cm, Theta : %f Degrees", x_cm, y_cm, theta_deg)
    
    #line = ser.readline().decode('utf-8').strip() 
    #print("a")
    #print(line)

    talker(int(x_cm), int(y_cm), int(theta_deg))

def listener():
    rospy.init_node('hector_slam_listener', anonymous=True)
    
    rospy.Subscriber("/slam_out_pose", PoseStamped, pose_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

