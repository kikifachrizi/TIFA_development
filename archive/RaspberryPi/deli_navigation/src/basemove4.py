#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import serial


def move_robot():

    rospy.init_node('move_robot', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    move_cmd = Twist()
    move_cmd.linear.x = 1.0

    duration = 20  # detik
    rate = rospy.Rate(10)

    start_time = rospy.get_time()

    ser = serial.Serial('/dev/ttyUSB1', 57600, timeout=1)

    while rospy.get_time() - start_time < duration:
        pub.publish(move_cmd)
        message = f"{round(move_cmd.linear.x , 2)};{round(0.0)};{round(0.0)};\n"

        ser.write(message.encode('utf-8'))
        rate.sleep()

    move_cmd.linear.x = 0
    pub.publish(move_cmd)

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
