#!/usr/bin/env python3
import rospy
import numpy as np
import pandas as pd
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point, PoseStamped
from sklearn.neighbors import NearestNeighbors
from scipy.spatial import KDTree

navigation_data = pd.read_csv('../maps/path_points.csv')
navigation_points = navigation_data[['x', 'y']].values

def compute_centerline(points, k=30):
    knn = NearestNeighbors(n_neighbors=k)
    knn.fit(points)
    centerline = []
    for p in points:
        neighbors = knn.kneighbors([p], return_distance=False)[0]
        centroid = np.mean(points[neighbors], axis=0)
        centerline.append(centroid)
    return np.array(centerline)

centerline_points = compute_centerline(navigation_points)
tree_center = KDTree(centerline_points)

latest_pose = np.array([0.0, 0.0])
goal_received = False
goal_point = None
pub = None

def pose_callback(msg):
    global latest_pose
    latest_pose = np.array([msg.pose.position.x, msg.pose.position.y])

def goal_callback():
    global goal_received, goal_point
    goal_received = True
    #goal_point = np.array([msg.x, msg.y])
    goal_point = ([11.0, -2.5])
    rospy.loginfo("Received goal: {}".format(goal_point))

def bfs_centered_path(start_idx, goal_idx, points, k=5):
    knn = NearestNeighbors(n_neighbors=k)
    knn.fit(points)
    queue = [[start_idx]]
    visited = set()

    while queue:
        path = queue.pop(0)
        node = path[-1]
        if node == goal_idx:
            return [points[i] for i in path]
        if node not in visited:
            visited.add(node)
            neighbors = knn.kneighbors([points[node]], return_distance=False)[0]
            for neighbor in neighbors:
                if neighbor not in visited:
                    queue.append(path + [neighbor])
    return []

def smooth_path(corrected_path):
    smoothed_path = []
    prev_point = None
    for i in range(0, len(corrected_path), 5):
        sample = corrected_path[i:i+5]
        avg_x = np.mean(sample[:, 0]).round(1)
        avg_y = np.mean(sample[:, 1]).round(1)
        current_point = [avg_x, avg_y, 0.0]
        if prev_point is None or np.linalg.norm(np.array(current_point) - np.array(prev_point)) > 0.3:
            smoothed_path.append(current_point)
            prev_point = current_point
    return np.array(smoothed_path)

if __name__ == '__main__':
    rospy.init_node('knn_path_generator_node')
    pub = rospy.Publisher('/path_ml', Float32MultiArray, queue_size=10)

    rospy.Subscriber('/slam_out_pose', PoseStamped, pose_callback)
    #rospy.Subscriber('/goal', Point, goal_callback)
    goal_callback()
    rate = rospy.Rate(1.0) 
    rospy.loginfo("Node started. Waiting for /goal...")

    while not rospy.is_shutdown():
        if goal_received:
            start_idx = tree_center.query(latest_pose)[1]
            goal_idx = tree_center.query(goal_point)[1]

            corrected_path = bfs_centered_path(start_idx, goal_idx, centerline_points, k=5)

            if corrected_path:
                corrected_path = np.array(corrected_path)
                smoothed = smooth_path(corrected_path)
                flat_path = smoothed.flatten().tolist()
                pub.publish(Float32MultiArray(data=flat_path))
                rospy.loginfo("Published path with {} points.".format(len(smoothed)))
            else:
                rospy.logwarn("No path found.")

            goal_received = False  
        rate.sleep()
