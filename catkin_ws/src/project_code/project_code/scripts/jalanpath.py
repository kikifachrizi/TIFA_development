#!/usr/bin/env python3

import rospy
import serial
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def send_to_mcu(linear_x, angular_z):
    message = f"{round(linear_x, 2)};{round(angular_z, 2)};\n"
    ser.write(message.encode('utf-8'))
    rospy.loginfo(f"Sent to MCU: {message.strip()}")

def cmd_vel_callback(msg):
    linear_x = msg.linear.x
    angular_z = msg.angular.z
    send_to_mcu(linear_x, angular_z)

if __name__ == '__main__':
    rospy.init_node('cmd_vel_to_serial', anonymous=True)
    
    # Sesuaikan port dan baudrate sesuai dengan mikrokontroler Anda
    ser = serial.Serial('/dev/ttyUSB1', 57600, timeout=1) 
    
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    
    rospy.loginfo("Listening to /cmd_vel and sending to MCU...")
    rospy.spin()

