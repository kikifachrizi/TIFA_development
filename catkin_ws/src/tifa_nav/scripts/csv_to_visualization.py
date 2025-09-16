#!/usr/bin/env python3
import rospy
import csv
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def publish_trajectory():
    rospy.init_node('csv_to_visual', anonymous=True)
    
    pub_data = rospy.Publisher('/new_set', Float32MultiArray, queue_size=10)
    pub_marker = rospy.Publisher('/trajectory_marker', Marker, queue_size=10)

    msg = Float32MultiArray()
    data_list = []
    points = []

    # Baca CSV dan simpan ke list + marker
    with open('/home/tifa/catkin_ws/src/tifa_nav/maps/RutePameran.csv', 'r') as f:
        reader = csv.reader(f)
        rows = list(reader)
        
        for row in rows[1:]:
            floats = [float(val) for val in row]
            data_list.extend(floats)
            if len(floats) >= 2:  
                pt = Point()
                pt.x = floats[0]
                pt.y = -1*floats[1]
                pt.z = 0.0
                points.append(pt)

    msg.data = data_list

    # Buat marker
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "trajectory"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.05
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.points = points

    # Loop publish
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub_marker.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_trajectory()
    except rospy.ROSInterruptException:
        pass
