#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty

# Membaca input dari keyboard (non-blocking)
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        if key == '':
            key = None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

# Fungsi utama teleoperasi
def teleop_key():
    # Inisialisasi node ROS
    rospy.init_node('teleop_key', anonymous=True)

    # Publisher untuk cmd_vel
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Kecepatan dasar
    speed = 0.35 #maks 10.91 rad/s atau 0.98 m/s
    turn = 1.0 #maks 10.91 rad/s atau 0.98 m/s

    # Membuat pesan Twist kosong
    move_cmd = Twist()

    print("Control robot dengan tombol keyboard:")
    print("w: maju, x: mundur, a: belok kiri, d: belok kanan, q: berhenti")

    # Loop teleop
    while not rospy.is_shutdown():
        key = get_key()  # Membaca tombol secara non-blocking

        # Mengatur gerakan berdasarkan input
        if key == 'w':
            move_cmd.linear.x = speed
            move_cmd.linear.y = 0
            move_cmd.angular.z = 0
        elif key == 'x':
            move_cmd.linear.x = -speed
            move_cmd.linear.y = 0
            move_cmd.angular.z = 0
        elif key == 'a':
            move_cmd.linear.x = 0
            move_cmd.linear.y = speed
            move_cmd.angular.z = 0
        elif key == 'd':
            move_cmd.linear.x = 0
            move_cmd.linear.y = -speed
            move_cmd.angular.z = 0
        elif key == 'e':
            move_cmd.linear.x = speed/2
            move_cmd.linear.y = -speed/2
            move_cmd.angular.z = 0
        elif key == 'q':
            move_cmd.linear.x = speed/2
            move_cmd.linear.y = speed/2
            move_cmd.angular.z = 0
        elif key == 'c':
            move_cmd.linear.x = -speed/2
            move_cmd.linear.y = -speed/2
            move_cmd.angular.z = 0
        elif key == 'z':
            move_cmd.linear.x = -speed/2
            move_cmd.linear.y = speed/2
            move_cmd.angular.z = 0

        # Rotasi
        elif key == 'l':
            move_cmd.linear.x = 0
            move_cmd.linear.y = 0
            move_cmd.angular.z = turn
        elif key == 'k':
            move_cmd.linear.x = 0
            move_cmd.linear.y = 0
            move_cmd.angular.z = -turn
        else:
            move_cmd.linear.x = 0
            move_cmd.linear.y = 0
            move_cmd.angular.z = 0

        # Publikasikan perintah gerakan
        pub.publish(move_cmd)

        # Jika 'p' ditekan, keluar dari program
        if key == 'p':
            break

    print("Program selesai.")
    rospy.spin()

if __name__ == "__main__":
    try:
        teleop_key()
    except rospy.ROSInterruptException:
        pass
