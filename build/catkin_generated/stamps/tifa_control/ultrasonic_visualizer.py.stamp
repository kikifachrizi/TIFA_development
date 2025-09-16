#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion

# Sudut sensor ultrasonik dalam radian (sensor setiap 30°)
sensor_angles = np.radians([-15,-30,-60,-75,-105,-120,-150,-165,-195,-210,-240,-255,-285,-300,-330,-315])
num_sensors = len(sensor_angles)

# Jarak barier (dalam meter)
barrier_distance = 0.40  # 30 cm

def ultrasonic_callback(msg):
    if len(msg.data) != num_sensors:
        rospy.logwarn("Received data length does not match expected number of sensors!")
        return

    # Marker untuk titik deteksi ultrasonik
    point_marker = Marker()
    point_marker.header.frame_id = "base_link"
    point_marker.header.stamp = rospy.Time.now()
    point_marker.type = Marker.POINTS  
    point_marker.action = Marker.ADD
    point_marker.scale.x = 0.1  # Ukuran titik
    point_marker.scale.y = 0.1
    point_marker.color.a = 1.0
    point_marker.color.r = 1.0  # Titik berwarna merah
    point_marker.color.g = 0.0
    point_marker.color.b = 0.0
    point_marker.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

    # Marker untuk garis antar titik
    line_marker = Marker()
    line_marker.header.frame_id = "base_link"
    line_marker.header.stamp = rospy.Time.now()
    line_marker.type = Marker.LINE_STRIP  
    line_marker.action = Marker.ADD
    line_marker.scale.x = 0.05  
    line_marker.color.a = 1.0
    line_marker.color.r = 0.0
    line_marker.color.g = 1.0  # Hijau
    line_marker.color.b = 0.0
    line_marker.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

    # Marker untuk barier (lingkaran 20 cm)
    barrier_marker = Marker()
    barrier_marker.header.frame_id = "base_link"
    barrier_marker.header.stamp = rospy.Time.now()
    barrier_marker.type = Marker.LINE_STRIP
    barrier_marker.action = Marker.ADD
    barrier_marker.scale.x = 0.03  # Lebar garis barier
    barrier_marker.color.a = 1.0
    barrier_marker.color.r = 1.0  # Warna merah default
    barrier_marker.color.g = 0.5  # Warna hijau jika aman
    barrier_marker.color.b = 0.0
    barrier_marker.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

    # Buat lingkaran barier
    num_barrier_points = 50
    for i in range(num_barrier_points + 1):
        angle = 2 * np.pi * i / num_barrier_points
        x = barrier_distance * np.cos(angle)
        y = barrier_distance * np.sin(angle)
        barrier_marker.points.append(Point(x=x, y=y, z=0))

    # Periksa apakah ada objek dalam zona barier
    object_inside_barrier = False

    for i in range(num_sensors):
        distance = msg.data[i] / 100.0  # Konversi cm ke meter
        angle = sensor_angles[i]

        # Konversi ke koordinat Cartesian
        x = distance * np.cos(angle)
        y = distance * np.sin(angle)
        point = Point(x=x, y=y, z=0)

        # Tambahkan titik ke marker titik dan garis
        point_marker.points.append(point)
        line_marker.points.append(point)


        # Jika ada objek dalam zona barier, ubah warna barier
        if distance <= barrier_distance:
            object_inside_barrier = True

    # Opsional: Tambahkan titik pertama ke akhir garis untuk menutup bentuk
    if len(line_marker.points) > 0:
        line_marker.points.append(line_marker.points[0])

    # Ubah warna barier jika ada objek dalam zona
    if object_inside_barrier:
        barrier_marker.color.r = 1.0  # Warna merah terang
        barrier_marker.color.g = 0.0  # Tidak ada warna hijau
    else:
        barrier_marker.color.r = 1.0  # Warna merah biasa
        barrier_marker.color.g = 0.5  # Warna hijau lembut

    # Publikasikan marker ke RViz
    pub_points.publish(point_marker)
    pub_lines.publish(line_marker)
    pub_barrier.publish(barrier_marker)

# Inisialisasi node ROS
rospy.init_node('ultrasonic_visualizer')

# Publisher untuk visualisasi di RViz
pub_points = rospy.Publisher('/ultrasonic_points', Marker, queue_size=10)
pub_lines = rospy.Publisher('/ultrasonic_lines', Marker, queue_size=10)
pub_barrier = rospy.Publisher('/ultrasonic_barrier', Marker, queue_size=10)

# Subscriber untuk data sensor ultrasonik
rospy.Subscriber('/ultrasonic_data', Float32MultiArray, ultrasonic_callback)

rospy.spin()
