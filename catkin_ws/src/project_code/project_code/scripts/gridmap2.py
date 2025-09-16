#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker

# Publisher for the marker to RViz
marker_pub = None

def calculate_grid_position(x, y, resolution):
    """
    Menghitung posisi baris dan kolom dari koordinat x, y berdasarkan resolusi peta.
    """
    row = math.floor(y / resolution)
    col = math.floor(x / resolution)
    return row, col

def create_grid_marker(row, col, resolution):
    """
    Membuat marker di posisi grid tertentu untuk ditampilkan di RViz.
    """
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "grid"
    marker.id = row * 1000 + col  # ID unik untuk setiap marker
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position.x = col * resolution + resolution / 2.0
    marker.pose.position.y = row * resolution + resolution / 2.0
    marker.pose.position.z = 0.5  # Sedikit di atas tanah
    marker.pose.orientation.w = 1.0

    # Ukuran marker (ukuran teks)
    marker.scale.z = 0.1

    # Warna marker (teks merah)
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    # Teks marker
    marker.text = "Row: {}, Col: {}".format(row, col)

    return marker

def map_callback(map_data):
    """
    Callback function untuk memproses data peta OccupancyGrid dan membuat marker di RViz.
    """
    global marker_pub
    resolution = map_data.info.resolution  # Resolusi peta dalam meter/grid

    # Iterasi melalui semua cell di OccupancyGrid
    width = map_data.info.width
    height = map_data.info.height
    origin_x = map_data.info.origin.position.x
    origin_y = map_data.info.origin.position.y

    for y in range(height):
        for x in range(width):
            # Mendapatkan nilai occupancy (misalnya 0 untuk free, 100 untuk obstacle)
            index = x + y * width
            occupancy_value = map_data.data[index]

            # Hanya buat marker untuk cell yang valid (tidak unknown)
            if occupancy_value != -1:
                # Menghitung posisi cell di koordinat dunia
                world_x = origin_x + x * resolution
                world_y = origin_y + y * resolution

                # Hitung baris dan kolom dari posisi grid
                row, col = calculate_grid_position(world_x, world_y, resolution)

                # Buat marker untuk posisi grid ini
                marker = create_grid_marker(row, col, resolution)

                # Publikasikan marker ke RViz
                marker_pub.publish(marker) # type: ignore
    rospy.loginfo("res %f" % resolution)

def main():
    global marker_pub

    # Inisialisasi node ROS
    rospy.init_node('map_grid_marker_node')

    # Membuat publisher untuk mengirimkan marker ke RViz
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    # Subscribe ke topik peta (OccupancyGrid)
    rospy.Subscriber('/map', OccupancyGrid, map_callback)

    # Menjalankan node
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
