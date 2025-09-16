#!/usr/bin/env python3

import rospy
import serial
import threading
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  #ftdi


def cmd_vel_callback(msg):

    # rospy.loginfo(
    # "Sending Data: x=%.2f, y=%.2f, z=%.2f",
    # round(msg.linear.x, 2),
    # round(msg.linear.y, 2),
    # round(msg.angular.z, 2),
    # )

    message = f"{round(msg.linear.x, 2)} {round(msg.linear.y, 2)} {round(msg.angular.z, 2)};\n"
    ser.write(message.encode('utf-8'))


def main():
    global pub
    rospy.init_node('manual_control', anonymous=True)
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)


    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
