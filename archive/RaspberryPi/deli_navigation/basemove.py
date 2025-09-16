#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import tf

def send_goal(x, y, theta):
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(1)

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()

    goal.pose.position.x = x
    goal.pose.position.y = y
    quat = tf.transformations.quaternion_from_euler(0, 0, theta)
    goal.pose.orientation.x = quat[0]
    goal.pose.orientation.y = quat[1]
    goal.pose.orientation.z = quat[2]
    goal.pose.orientation.w = quat[3]

    print("x : ", goal.pose.position.x)
    print("y : ", goal.pose.position.y)

    pub.publish(goal)
    #rospy.loginfo(f"Goal sent: x={x}, y={y}, theta={theta}")

if __name__ == "__main__":
    rospy.init_node('send_goal_node')

    waypoints = [
        {"x": 1.0, "y": 2.0, "theta": 0.0},
        {"x": -1.0, "y": 0.5, "theta": 1.57},
        {"x": 0.0, "y": 0.0, "theta": -1.57}
    ]

    for waypoint in waypoints:
        send_goal(waypoint['x'], waypoint['y'], waypoint['theta'])
        rospy.sleep(10)