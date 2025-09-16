#!/usr/bin/env python3

import rospy
import csv
from geometry_msgs.msg import PoseStamped
import time
import threading


data_count = 0
running = True 

def pose_callback(msg):
    global data_count, running
    if not running:
        return

    x = msg.pose.position.x
    y = msg.pose.position.y
    data_count += 1  

    print("x: ", x)
    print("y: ", y)

    with open('trajectory.csv', mode='a') as file:
        writer = csv.writer(file)
        writer.writerow([x, y])

def stop_after_duration(duration):
    global running
    time.sleep(duration)  
    running = False
    rospy.signal_shutdown("Timer expired") 

def listener():
    global running, data_count

    rospy.init_node('slam_to_csv', anonymous=True)
    
    with open('trajectory.csv', mode='w') as file:
        writer = csv.writer(file)
        writer.writerow(['x', 'y'])


    rospy.Subscriber('/slam_out_pose', PoseStamped, pose_callback)

    
    timer_thread = threading.Thread(target=stop_after_duration, args=(1,))
    timer_thread.start()

    rospy.spin()

    print(f"Total data points received in 1 second: {data_count}")

if __name__ == '__main__':
    listener()
