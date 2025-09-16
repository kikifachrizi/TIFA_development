#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import serial

class RobotNavigator:
    def __init__(self):
        rospy.init_node('robot_navigator', anonymous=True)

        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.pose_sub = rospy.Subscriber('/slam_out_pose', PoseStamped, self.pose_callback)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

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

    def cmd_vel_callback(self, msg):
        linear_velocity_x = msg.linear.x
        linear_velocity_y = msg.linear.y
        angular_velocity = msg.angular.z

        ser = serial.Serial('/dev/ttyUSB1', 57600, timeout=1)
        message = f"{round(linear_velocity_x, 2)};{round(linear_velocity_y,2)};{round(angular_velocity, 2)};\n"
        ser.write(message.encode('utf-8'))
        
        self.vel_pub.publish(msg)

    def send_goal(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0

        rospy.loginfo(f"Sending goal: x={x}, y={y}")
        goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        rospy.sleep(1)
        goal_pub.publish(goal)

        self.goal_position = (x, y)

    def main_loop(self):
        rospy.loginfo("Robot navigator running...")
        while not rospy.is_shutdown():
            if self.current_pose:
                rospy.loginfo_throttle(5, f"Current Pose: {self.current_pose}")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        navigator = RobotNavigator()

        navigator.send_goal(x=2.0, y=0.0)

        navigator.main_loop()
    except rospy.ROSInterruptException:
        pass
