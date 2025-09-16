#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import csv

# Callback untuk topik /move_base/NavfnROS/plan
def plan_callback(data):
    rospy.loginfo("Received a plan message")
    with open('plan_xy33.csv', 'a') as csvfile:
        writer = csv.writer(csvfile)
        for pose in data.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            rospy.loginfo(f"Plan - x: {x}, y: {y}")
            writer.writerow([x, y])

# Callback untuk topik /trajectory
# def trajectory_callback(data):
#     rospy.loginfo("Received a trajectory message")
#     with open('trajectory_xy.csv', 'a') as csvfile:
#         writer = csv.writer(csvfile)
#         for pose in data.poses:
#             x = pose.pose.position.x
#             y = pose.pose.position.y
#             rospy.loginfo(f"Trajectory - x: {x}, y: {y}")
#             writer.writerow([x, y])

def listener():
    # Inisialisasi node ROS
    rospy.init_node('xy_listener', anonymous=True)

    # Subscriber untuk topik /move_base/NavfnROS/plan
    rospy.Subscriber('/move_base/NavfnROS/plan', Path, plan_callback)

    # Subscriber untuk topik /trajectory
    # rospy.Subscriber('/trajectory', Path, trajectory_callback)

    # ROS akan tetap berjalan hingga dihentikan
    rospy.spin()

if __name__ == '__main__':
    try:
        # Buat file kosong untuk menyimpan data
        with open('plan_xy33.csv', 'w') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['x', 'y'])

        # with open('trajectory_xy.csv', 'w') as csvfile:
        #     writer = csv.writer(csvfile)
        #     writer.writerow(['x', 'y'])

        # Jalankan node
        listener()
    except rospy.ROSInterruptException:
        pass
