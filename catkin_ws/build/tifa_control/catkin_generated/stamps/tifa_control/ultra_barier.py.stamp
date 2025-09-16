#!/usr/bin/env python3
import rospy
import serial
import numpy as np
from std_msgs.msg import Float32MultiArray

# Konfigurasi Serial
serial_port = "/dev/ttyACM0"  # Sesuaikan dengan port ESP32-S3
baud_rate = 115200

num_sensors = 12  # Jumlah sensor ultrasonik

def read_ultrasonic_data():
    rospy.init_node("serial_ultrasonic_reader", anonymous=True)
    pub = rospy.Publisher("/ultrasonic_data", Float32MultiArray, queue_size=10)

    ser = serial.Serial(serial_port, baud_rate, timeout=1)

    while not rospy.is_shutdown():
        try:
            line = ser.readline().decode("utf-8").strip()
            if line:
                distances = list(map(float, line.split(",")))

                if len(distances) == num_sensors:
                    msg = Float32MultiArray()
                    msg.data = distances
                    pub.publish(msg)
                    rospy.loginfo(f"Published: {distances}")

        except Exception as e:
            rospy.logerr(f"Error reading serial: {e}")

if __name__ == "__main__":
    read_ultrasonic_data()
