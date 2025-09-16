#!/usr/bin/env python3

import rospy
import serial
from geometry_msgs.msg import Twist
from std_msgs.msg import String

ser = serial.Serial('/dev/ttyUSB1', 57600, timeout=1) 

def cmd_vel_callback(msg):

    rospy.loginfo("Velocities: x = %.2f, y = %.2f, z = %.2f", 
                  msg.linear.x, msg.linear.y, msg.angular.z)
    
    x = msg.linear.x
    z = msg.angular.z

    # if x < 0:
    #     x = x - 1
    # if x > 0:
    #     x = x + 2
    if z < 0:
        z = z + 1.2
    if z > 0:
        z = z - 1.2

    rospy.loginfo("Velocities 2 : x = %.2f, y = %.2f, z = %.2f", 
                  x, msg.angular.y, z)
    
    message = f"{round(x, 2)};{round(msg.linear.y,2)};{round(z,2)};\n"  

    ser.write(message.encode('utf-8'))

def main():
    rospy.init_node('manual_control', anonymous=True)
    rospy.Subscriber('/turtle1/cmd_vel', Twist, cmd_vel_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
