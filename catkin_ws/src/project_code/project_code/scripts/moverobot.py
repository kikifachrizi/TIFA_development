#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client(x_goal, y_goal):

    # Initialize the node
    rospy.init_node('movebase_client')

    # Create an action client
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Wait until the action server is available
    client.wait_for_server()

    # Create a new goal with x, y positions
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x_goal
    goal.target_pose.pose.position.y = y_goal
    goal.target_pose.pose.orientation.w = 1.0

    # Send the goal to the server
    client.send_goal(goal)

    # Wait for the result
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        # Move to the goal coordinates (x, y)
        result = movebase_client(1.0, 1.0)
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
