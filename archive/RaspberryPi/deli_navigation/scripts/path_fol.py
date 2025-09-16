#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal
import actionlib

class FollowTrajectory:
    def __init__(self):
        rospy.init_node('trajectory_follower')

        self.path_subscriber = rospy.Subscriber('/visualization_path', Path, self.path_callback)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        rospy.loginfo("Wait")
        self.client.wait_for_server()

        self.path = [] 

    def path_callback(self, msg):
        self.path = msg.poses
        rospy.loginfo("Received new path with %d points", len(self.path))

        self.follow_path()

    def follow_path(self):
        if not self.path:
            rospy.logwarn("No path to follow.")
            return

        for pose in self.path:
            goal = MoveBaseActionGoal()
            goal.goal.target_pose = pose

            rospy.loginfo("Sending goal to move_base: %s", pose)

            self.client.send_goal(goal.goal)

            self.client.wait_for_result()

            if not self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.logwarn("Goal failed, stopping path follow.")
                break
            rospy.loginfo("Reached goal: %s", pose)

if __name__ == '__main__':
    try:
        trajectory_follower = FollowTrajectory()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS InterruptException")
