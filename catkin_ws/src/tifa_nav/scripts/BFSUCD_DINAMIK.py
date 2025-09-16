#!/usr/bin/env python3
import rospy
import numpy as np
import pandas as pd
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped as PathPose
from sklearn.neighbors import NearestNeighbors
from scipy.spatial import KDTree

navigation_data = pd.read_csv(r"~/catkin_ws/src/tifa_nav/maps/tifa/path_points_tes_Del.csv")  # Ganti path sesuai lokasi file
navigation_points = navigation_data[['x', 'y']].values

centerline_points = None
tree_center = None
current_pose = None
obstacles = []
smoothed_path = []
goal_point = np.array([7.4, 1.8])

path_pub = None
target_pub = None

def compute_centerline(points, k=30):
    knn = NearestNeighbors(n_neighbors=k)
    knn.fit(points)
    centerline = []
    for p in points:
        neighbors = knn.kneighbors([p], return_distance=False)[0]
        centroid = np.mean(points[neighbors], axis=0)
        centerline.append(centroid)
    return np.array(centerline)

def bfs_shortest_path_with_knn(points, start_idx, goal_idx, k=5, blocked_indices=set()):
    knn = NearestNeighbors(n_neighbors=k)
    knn.fit(points)
    dists_all, indices_all = knn.kneighbors(return_distance=True)

    import heapq
    queue = [(0, start_idx, [start_idx])]
    visited = set()

    while queue:
        cost, current_idx, path = heapq.heappop(queue)
        if current_idx == goal_idx:
            return [points[i] for i in path]
        if current_idx in visited or current_idx in blocked_indices:
            continue
        visited.add(current_idx)
        dists = dists_all[current_idx]
        neighbors = indices_all[current_idx]
        for dist, neighbor in zip(dists, neighbors):
            if neighbor not in visited and neighbor not in blocked_indices:
                new_cost = cost + dist
                new_path = path + [neighbor]
                heapq.heappush(queue, (new_cost, neighbor, new_path))

    return []

def update_path():
    print("Update Path")
    global smoothed_path
    if current_pose is None:
        return

    blocked_indices = set()
    for i, point in enumerate(centerline_points):
        for obs in obstacles:
            if np.linalg.norm(point - obs) < 0.25:
                blocked_indices.add(i)
                break

    print("Corrected Path")
    start_idx = tree_center.query(current_pose[:2])[1]
    goal_idx = tree_center.query(goal_point)[1]
    corrected_path = bfs_shortest_path_with_knn(centerline_points, start_idx, goal_idx, k=5, blocked_indices=blocked_indices)

    # Smoothing path
    print("Smoothing Path")
    smoothed_path = []
    prev_point = None
    for i in range(0, len(corrected_path), 5):
        sample = corrected_path[i:i+5]
        if len(sample) == 0:
            continue
        avg_x = np.mean(np.array(sample)[:, 0]).round(2)
        avg_y = np.mean(np.array(sample)[:, 1]).round(2)
        current_point = [avg_x, avg_y, 0.0]
        if prev_point is None or np.linalg.norm(np.array(current_point) - np.array(prev_point)) > 0.3:
            smoothed_path.append(current_point)
            prev_point = current_point
    smoothed_path = np.array(smoothed_path)

    publish_path()
    publish_target()

def is_path_blocked():
    if len(smoothed_path) == 0:
        return False
    for point in smoothed_path[:10]:
        for obs in obstacles:
            if np.linalg.norm(np.array(point[:2]) - obs) < 0.25:
                return True
    return False

def pose_callback(msg):
    global current_pose
    current_pose = np.array([
        msg.pose.position.x,
        msg.pose.position.y,
        0.0
    ])

def scan_callback(msg):
    global obstacles
    if current_pose is None:
        return

    angle = msg.angle_min
    local_obstacles = []
    for r in msg.ranges:
        if msg.range_min < r < msg.range_max:
            x = r * np.cos(angle) + current_pose[0]
            y = r * np.sin(angle) + current_pose[1]
            local_obstacles.append(np.array([x, y]))
        angle += msg.angle_increment
    obstacles = local_obstacles

    if is_path_blocked():
        rospy.loginfo("Obstacle detected! Replanning path...")
        update_path()

def publish_path():
    path_msg = Path()
    path_msg.header.frame_id = "map"
    for pt in smoothed_path:
        pose = PathPose()
        pose.pose.position.x = pt[0]
        pose.pose.position.y = pt[1]
        pose.pose.position.z = 0.0
        path_msg.poses.append(pose)
    path_pub.publish(path_msg)

def publish_target():
    if len(smoothed_path) == 0:
        return
    next_point = smoothed_path[0]
    target_msg = PointStamped()
    target_msg.header.frame_id = "map"
    target_msg.point.x = next_point[0]
    target_msg.point.y = next_point[1]
    target_msg.point.z = 0.0
    target_pub.publish(target_msg)

if __name__ == "__main__":
    rospy.init_node("dynamic_replan_node")
    rospy.Subscriber("/slam_out_pose", PoseStamped, pose_callback)
    rospy.Subscriber("/scan", LaserScan, scan_callback)

    path_pub = rospy.Publisher("/planned_path", Path, queue_size=10)
    target_pub = rospy.Publisher("/target_point", PointStamped, queue_size=10)

    rospy.loginfo("Computing centerline and preparing KDTree...")
    centerline_points = compute_centerline(navigation_points, k=30)
    tree_center = KDTree(centerline_points)

    while current_pose is None and not rospy.is_shutdown():
        rospy.loginfo("Waiting for initial pose from /slam_out_pose...")
        rospy.sleep(0.5)

    rospy.loginfo("Received initial pose, computing first path...")
    update_path()

    rospy.spin()
