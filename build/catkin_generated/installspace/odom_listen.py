#!/usr/bin/env python3

import rospy
import serial
import threading
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Quaternion
from tf.transformations import quaternion_from_euler

ser2 = serial.Serial('/dev/ttyUSB2', 19200, timeout=1)  

data = ""
pub = None


def read_serial_data():
    global data
    while not rospy.is_shutdown():
        if ser2.in_waiting > 0:
            data = ser2.readline().decode('utf-8')
            # rospy.loginfo(f"Received from Teensy: {data}")
            publish_odometry(data)


def publish_odometry(data):
    try:
        values = data.split(';')
        
        encoder_value_5 = float(values[-3])  # X
        encoder_value_6 = float(values[-2])  # Y
        gyroZ = float(values[-1])  # Theta (Orientasi)

        # rospy.loginfo(
        # "parsing data: x=%.2f, y=%.2f, z=%.2f",
        # round(encoder_value_5, 2),
        # round(encoder_value_6, 2),
        # round(gyroZ, 2),
        # )

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map"

        odom.pose.pose.position.x = -1*encoder_value_5
        odom.pose.pose.position.y = encoder_value_6
        odom.pose.pose.position.z = 0  # Robot 2D, z = 0

        roll, pitch, yaw = 0, 0, gyroZ  

        quaternion = quaternion_from_euler(roll, pitch, yaw)
        odom.pose.pose.orientation = Quaternion(*quaternion)

        odom.twist.twist.linear.x = 0.0  # Misalnya, tidak ada kecepatan linear yang diberikan
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = gyroZ  # Kecepatan angular berdasarkan gyroZ

        # Publikasikan pesan Odometry
        pub.publish(odom)
        rospy.loginfo(f"Published Odometry: {odom}")
    except ValueError as e:
        rospy.logwarn(f"Error parsing data: {e}")


def main():
    global pub
    rospy.init_node('odometry_publisher', anonymous=True)
    pub = rospy.Publisher("/wheel_odom", Odometry, queue_size=10)
    thread = threading.Thread(target=read_serial_data, daemon=True)
    thread.start()

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
