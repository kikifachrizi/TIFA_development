#!/usr/bin/env python
import rospy
import tf
from tf.transformations import euler_from_quaternion
import time

rospy.init_node('get_tf_example')

# Membuat listener untuk mendapatkan transformasi
listener = tf.TransformListener()

while True:
    # Menunggu hingga transformasi tersedia
    listener.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(3.0))

    # Mendapatkan transformasi
    (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))

    # trans adalah tuple (x, y, z)
    print("Translation: ", trans)
    time.sleep(1)

    # rot adalah quaternion (x, y, z, w)
    # print("Rotation (Quaternion): ", rot)

    # # Jika Anda perlu mengonversi rotasi menjadi Euler angles
    # roll, pitch, yaw = euler_from_quaternion(rot)
    # print("Euler angles (roll, pitch, yaw): ", roll, pitch, yaw)

# except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#     rospy.logerr("Transform not available!")
