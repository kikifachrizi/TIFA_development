#!/usr/bin/env python3

import rospy
import serial
import threading
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

ser2 = serial.Serial('/dev/ttyUSB2', 19200, timeout=1)  #cp210x

data =""
pub = None


def read_serial_data():
    global data
    while not rospy.is_shutdown():
        if ser2.in_waiting > 0:
            data = ser2.readline().decode('utf-8') 
            rospy.loginfo(f"Received from Teensy: {data}")
            pub.publish(data)



def main():
    global pub
    rospy.init_node('pid_record', anonymous=True)
    pub = rospy.Publisher("/raw_data_pid", String, queue_size=10)

    thread = threading.Thread(target=read_serial_data, daemon=True)
    thread.start()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
