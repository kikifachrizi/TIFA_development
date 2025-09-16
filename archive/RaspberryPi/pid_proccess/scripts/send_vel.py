#!/usr/bin/env python
import rospy
import serial
from geometry_msgs.msg import Twist
import time
import math

# Setup Serial connection
ser = serial.Serial('/dev/ttyUSB1', 57600, timeout=1)  # Sesuaikan dengan port dan baudrate Anda

# Callback untuk menerima data dari /cmd_vel
def cmd_vel_callback(msg):
    # Ambil kecepatan linear di tiga sumbu (x, y, z)
    linear_velocity_x = msg.linear.x
    linear_velocity_y = msg.linear.y
    linear_velocity_z = msg.linear.z
    #angular_velocity_x = msg.angular.x
   # angular_velocity_y = msg.angular.z
#    angular_velocity_y = math.degrees(angular_velocity_y)

    linear_velocity_x = "%.2f" % linear_velocity_x
    linear_velocity_y = "%.2f" % linear_velocity_y
    linear_velocity_z = "%.2f" % linear_velocity_z
#    angular_velocity_y = "0.5"
#    print(msg.angular.z)
 #   print(type(msg.angular.z))
    # Buat pesan yang akan dikirim melalui USB TTL
    # Format pesan, bisa disesuaikan dengan protokol perangkat Anda
    # Contoh: "<linear_x>,<linear_y>,<linear_z>,<angular_z>"
    message = f"{linear_velocity_x};{linear_velocity_y};{linear_velocity_z};\n"
    
    # Kirim pesan ke USB TTL
    ser.write(message.encode('utf-8'))
    rospy.loginfo(f"Sending to USB TTL: {message.strip()}")
    time.sleep(0.2)

# Fungsi utama
if __name__ == "__main__":
    # Inisialisasi node ROS
    rospy.init_node("serial_vel", anonymous=True)

    # Subscriber untuk topik cmd_vel
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)

    # Spin untuk menjaga node tetap berjalan
    rospy.spin()
