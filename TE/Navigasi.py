#!/usr/bin/env python3
import rospy
import numpy as np
import pandas as pd
import serial
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray
from sklearn.neighbors import NearestNeighbors
from scipy.spatial import KDTree

navigation_data = pd.read_csv(r"sidang1_white_Nav.csv")
navigation_points = navigation_data[['x', 'y']].values

centerline_points = None
tree_center = None
current_pose = None
last_known_pose_for_path = None
obstacles = []
smoothed_path = []

goal_points = [
    np.array([6.0, 2.0]),
    np.array([2.0, 5.0]),
    np.array([7.0, -1.0])
]
current_goal_idx = 0

marker_pub = None
target_pub = None
replan_path_pub = None
current_target_idx = 1

knn = None
dists_all = None
indices_all = None

ser = None

def compute_centerline(points, k=30):
    knn = NearestNeighbors(n_neighbors=k)
    knn.fit(points)
    centerline = []
    for p in points:
        neighbors = knn.kneighbors([p], return_distance=False)[0]
        centroid = np.mean(points[neighbors], axis=0)
        centerline.append(centroid)
    return np.array(centerline)

def bfs_and_knn(points, start_idx, goal_idx, dists_all, indices_all, blocked_indices=set()):
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
    print("[WARN] No path found!")
    return []

def update_path():
    global smoothed_path, current_target_idx, last_known_pose_for_path
    if current_pose is None:
        print("[WARN] Current pose not set yet!")
        return

    last_known_pose_for_path = np.copy(current_pose)
    print(f"[INFO] Updating path using pose: {last_known_pose_for_path}")

    blocked_indices = set()
    if len(obstacles) > 0:
        obs_tree = KDTree(obstacles)
        for i, point in enumerate(centerline_points):
            if len(obs_tree.query_ball_point(point, r=0.3)) > 0:
                blocked_indices.add(i)

    print(f"[INFO] {len(blocked_indices)} blocked points identified")

    start_idx = tree_center.query(last_known_pose_for_path[:2])[1]
    goal_idx = tree_center.query(goal_points[current_goal_idx])[1]

    corrected_path = bfs_and_knn(
        centerline_points, start_idx, goal_idx,
        dists_all, indices_all,
        blocked_indices=blocked_indices
    )

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

    print(f"[INFO] Smoothed path has {len(smoothed_path)} points")
    current_target_idx = 1

    publish_marker_path()
    replan_msg = Float32MultiArray()
    replan_msg.data = [0]
    replan_path_pub.publish(replan_msg)
    publish_target_point()

def is_path_blocked():
    if len(smoothed_path) == 0:
        return False
    for point in smoothed_path[:10]:
        for obs in obstacles:
            if np.linalg.norm(np.array(point[:2]) - obs) < 0.3:
                return True
    return False

def pose_callback(msg):
    global current_pose, current_target_idx, current_goal_idx
    current_pose = np.array([msg.pose.position.x, msg.pose.position.y, 0.0])

    if len(smoothed_path) == 0:
        return

    target_point = smoothed_path[current_target_idx]
    dist_to_target = np.linalg.norm(current_pose[:2] - target_point[:2])

    if dist_to_target < 0.5:
        if current_target_idx + 1 < len(smoothed_path):
            current_target_idx += 1
            rospy.loginfo(f"Reached point {current_target_idx}, sending next point...")
            publish_target_point()
        else:
            rospy.loginfo(f"Goal {current_goal_idx + 1} reached!")

            try:
                ser.write(b'1\n')
                rospy.loginfo("[SERIAL] Sent '1', waiting for '0'...")

                while not rospy.is_shutdown():
                    if ser.in_waiting > 0:
                        response = ser.readline().strip()
                        try:
                            decoded = response.decode('utf-8')
                            parts = [part.strip() for part in decoded.split(';')]
                            rospy.loginfo(f"[SERIAL] Received parts: {parts}")

                            if len(parts) > 2 and parts[2] == '1':
                                rospy.loginfo("[SERIAL] Proceeding to next goal (index ke-3 adalah 0)")
                                break
                        except Exception as e:
                            rospy.logwarn(f"[SERIAL] Failed to parse response: {e}")
                        # response = ser.readline().strip()
                        # rospy.loginfo(f"[SERIAL] Received: {response}")
                        # if response == b'0':
                        #     rospy.loginfo("[SERIAL] Proceeding to next goal")
                        #     break
                    rospy.sleep(0.1)
            except Exception as e:
                rospy.logwarn(f"[SERIAL] Error: {e}")

            if current_goal_idx + 1 < len(goal_points):
                current_goal_idx += 1
                rospy.loginfo(f"Proceeding to next goal: {goal_points[current_goal_idx]}")
                update_path()
            else:
                rospy.loginfo("All goals reached.")
    else:
        ser.write(b'0\n')
        rospy.loginfo("[SERIAL] Sent '0', Moving towards target point...")

def scan_callback(msg):
    global obstacles
    if current_pose is None:
        return

    angle = msg.angle_min
    local_obstacles = []
    for r in msg.ranges:
        if max(msg.range_min, 0.6) < r < min(msg.range_max, 2.0):
            x = r * np.cos(angle) + current_pose[0]
            y = r * np.sin(angle) + current_pose[1]
            local_obstacles.append(np.array([x, y]))
        angle += msg.angle_increment
    obstacles = local_obstacles

    if is_path_blocked():
        rospy.loginfo("Obstacle detected! Replanning path...")
        replan_msg = Float32MultiArray()
        replan_msg.data = [1]
        replan_path_pub.publish(replan_msg)
        update_path()

def publish_marker_path():
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "path"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.05
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.pose.orientation.w = 1.0

    for pt in smoothed_path:
        p = Point(x=pt[0], y=pt[1], z=pt[2])
        marker.points.append(p)

    if marker_pub:
        marker_pub.publish(marker)
        print(f"[RVIZ] Published marker with {len(marker.points)} points")

def publish_target_point():
    if len(smoothed_path) == 0:
        print("[WARN] No path to publish target point")
        return
    next_point = smoothed_path[current_target_idx]
    msg = Float32MultiArray()
    msg.data = [next_point[0], next_point[1] * -1, 0.0]  
    print(f"[TARGET] Published target point: {next_point}")
    target_pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("dynamic_path_visualizer")

    try:
        ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1) 
        rospy.loginfo("[SERIAL] Connected to /dev/ttyUSB0")
    except Exception as e:
        rospy.logerr(f"[SERIAL] Failed to connect: {e}")
        exit(1)

    rospy.Subscriber("/slam_out_pose", PoseStamped, pose_callback)
    rospy.Subscriber("/scan", LaserScan, scan_callback)

    marker_pub = rospy.Publisher("/planned_path_marker", Marker, queue_size=10)
    target_pub = rospy.Publisher("/target_point", Float32MultiArray, queue_size=10)
    replan_path_pub = rospy.Publisher("/replan_path", Float32MultiArray, queue_size=10)

    rospy.loginfo("Preparing centerline and KDTree...")
    print("[INIT] Loading path data...")
    centerline_points = compute_centerline(navigation_points, k=30)
    tree_center = KDTree(centerline_points)

    knn = NearestNeighbors(n_neighbors=5)
    knn.fit(centerline_points)
    dists_all, indices_all = knn.kneighbors(return_distance=True)

    while current_pose is None and not rospy.is_shutdown():
        rospy.loginfo("Waiting for initial pose...")
        rospy.sleep(0.5)

    last_known_pose_for_path = np.copy(current_pose)
    rospy.loginfo("Initial pose received. Generating path...")
    update_path()

    rospy.spin()
