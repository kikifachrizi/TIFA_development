#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import math
import serial

class RobotNavigator:
    def __init__(self):
        rospy.init_node('robot_navigator', anonymous=True)

        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.pose_sub = rospy.Subscriber('/slam_out_pose', PoseStamped, self.pose_callback)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        self.vel_pub = rospy.Publisher('/robot_velocity', Twist, queue_size=10)

        self.lidar_data = []
        self.current_pose = None
        self.map_data = None

        self.rate = rospy.Rate(10)
        self.goal_position = None

    def laser_callback(self, msg):
        self.lidar_data = msg.ranges
        rospy.loginfo_once("Lidar data received.")

    def pose_callback(self, msg):
        self.current_pose = msg.pose
        rospy.loginfo_once("Pose data received.")

    def map_callback(self, msg):
        self.map_data = msg.data
        rospy.loginfo_once("Map data received.")

    def send_goal(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0

        rospy.loginfo(f"Sending goal: x={x}, y={y}")
        goal_pub = rospy.Publisher('/trajectory', PoseStamped, queue_size=10)

        rospy.sleep(1)
        goal_pub.publish(goal)

        self.goal_position = (x, y)

    def calculate_linear_velocity(self):
        """Menghitung kecepatan linear berdasarkan jarak ke goal pada x dan y."""
        if self.current_pose and self.goal_position:
            x_current = self.current_pose.position.x
            y_current = self.current_pose.position.y
            x_goal, y_goal = self.goal_position

            distance_x = x_goal - x_current
            distance_y = y_goal - y_current
            distance = math.sqrt(distance_x**2 + distance_y**2)

            linear_velocity_x = min(distance_x, 2.0) 
            linear_velocity_y = min(distance_y, 2.0)

            rospy.loginfo(f"Linear velocity X: {linear_velocity_x:.2f} m/s, Linear velocity Y: {linear_velocity_y:.2f} m/s")
            return linear_velocity_x, linear_velocity_y
        return 0.0, 0.0

    def main_loop(self):
        rospy.loginfo("Robot navigator running...")
        ser = serial.Serial('/dev/ttyUSB1', 57600, timeout=1)
        rospy.loginfo("Terhubung ke STM32")
        while not rospy.is_shutdown():
            if self.current_pose:
                rospy.loginfo_throttle(5, f"Current Pose: {self.current_pose}")

            linear_velocity_x, linear_velocity_y = self.calculate_linear_velocity()
            twist = Twist()
            twist.linear.x = linear_velocity_x
            twist.linear.y = linear_velocity_y

            message = f"{round(twist.linear.x, 2)};{round(twist.linear.y,2)};{round(0.0)};\n"

            ser.write(message.encode('utf-8'))

            self.vel_pub.publish(twist)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        navigator = RobotNavigator()

        navigator.send_goal(x=2.0, y=0.0)

        navigator.main_loop()
    except rospy.ROSInterruptException:
        pass
