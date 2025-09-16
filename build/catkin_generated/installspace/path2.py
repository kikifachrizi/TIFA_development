#!/usr/bin/env python3

import rospy
import pandas as pd
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def publish_trajectory(csv_file):

    data = pd.read_csv(csv_file)

    rospy.init_node('trajectory_publisher', anonymous=True)
    pub = rospy.Publisher('/tutel', Path, queue_size=10)

    path = Path()
    path.header.frame_id = "map" 

    for index, row in data.iterrows():
        # if index >= 150 and index <= 200:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = row['x']
        pose.pose.position.y = row['y']
        pose.pose.position.z = 0
        path.poses.append(pose)
        
        #pub.publish(path)
        #print(index)
        #rospy.sleep(1)
    while not rospy.is_shutdown():
        pub.publish(path)
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        csv_file = 'trajectory_xy6.csv'
        publish_trajectory(csv_file)
    except rospy.ROSInterruptException:
        pass
