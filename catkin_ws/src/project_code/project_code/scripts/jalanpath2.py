#!/usr/bin/env python3

import time
import rospy
import serial
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# Inisialisasi serial untuk komunikasi dengan mikrokontroler
ser = serial.Serial('/dev/ttyUSB2', 57600, timeout=1)  # Sesuaikan dengan port Anda

# Inisialisasi publisher untuk cmd_vel
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def cmd_vel_callback(msg):
    """
    Callback untuk membaca data dari topik /cmd_vel dan mengirimkan ke mikrokontroler.
    """
    linear_x = msg.linear.x
    linear_y = msg.linear.y  # Tambahkan linear Y
    angular_z = msg.angular.z
    xv = 1.0
    yv = 1.0
    zv = 0.8

    # Format data untuk dikirim ke mikrokontroler
    message = f"{round(linear_x, 2)*xv} {round(linear_y, 2)*yv} {round(angular_z, 2)*yv};\n"
    
    # Kirim data ke mikrokontroler via Serial
    ser.write(message.encode('utf-8'))
    
    # Logging untuk debugging
    rospy.loginfo("Sent to MCU: X=%.2f | Y=%.2f | Z=%.2f", linear_x*xv, linear_y*yv, angular_z*zv)

def main():
    rospy.init_node('send_vel_to_mcu', anonymous=True)
    
    # Subscribe ke topik /cmd_vel untuk membaca perintah kecepatan
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    
    rospy.loginfo("Node send_vel_to_mcu started...")
    
    rospy.spin()

if __name__ == '__main__':
    main()
