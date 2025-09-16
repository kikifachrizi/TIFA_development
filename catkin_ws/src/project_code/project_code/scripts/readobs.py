#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid

def map_callback(data):
    width = data.info.width
    height = data.info.height
    grid = data.data
    for i in range(height):
        for j in range(width):
            cell = grid[i * width + j]
            if cell == 0:
                print(f"Cell ({i}, {j}) is free")
            elif cell == 100:
                print(f"Cell ({i}, {j}) is an obstacle")
            else:
                print(f"Cell ({i}, {j}) is unknown")

if __name__ == '__main__':
    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    rospy.spin()
