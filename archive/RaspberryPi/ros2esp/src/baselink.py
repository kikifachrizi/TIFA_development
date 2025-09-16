#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg
from tf2_geometry_msgs import PointStamped
import time

# Inisialisasi node
rospy.init_node('get_tf2_example')

# Membuat buffer dan listener
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

while True:
    # Menunggu dan mendapatkan transformasi dari 'base_link' ke 'map'
    transform = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(3.0))

    # Menampilkan hasil transformasi
    print("Translation: ", transform.transform.translation)
    #print("Rotation (Quaternion): ", transform.transform.rotation)
    time.sleep(1)
#except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#    rospy.logerr("Transform not available!")
