#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
import matplotlib.pyplot as plt

trajectory_x = []
trajectory_y = []

def path_callback(msg):
    global trajectory_x, trajectory_y
    trajectory_x = [pose.pose.position.x for pose in msg.poses]
    trajectory_y = [pose.pose.position.y for pose in msg.poses]

if __name__ == "__main__":
    rospy.init_node("trajectory_saver")
    rospy.Subscriber("/path", Path, path_callback)

    rospy.sleep(2)  # Tunggu data path diterima

    # Plot Trajectory
    plt.figure(figsize=(8, 8))
    plt.plot(trajectory_x, trajectory_y, label="Trajectory", color="blue")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.title("Robot Trajectory")
    plt.legend()
    plt.grid()

    # Save to Image
    plt.savefig("trajectory.png")
    plt.show()
