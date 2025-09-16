#!/usr/bin/env python3

import rospy
import csv
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import pandas as pd
import keyboard

stope = 0

def process_csv_with_averaging():
    input_file = 'Trajectory5.csv'
    output_file = 'T4_Inc20_Samp20.csv'

    data = pd.read_csv(input_file)

    if 'x' not in data.columns or 'y' not in data.columns:
        raise ValueError("Kolom 'x' atau 'y' tidak ditemukan dalam file CSV")

    data['x'] = data['x']
    data['y'] = data['y'] * -1
    data['th'] = data['th'] * -1

    averages = []
    for i in range(0, len(data), 20):
        sample = data.iloc[i:i+20]
        avg_x = sample['x'].mean().round(1)
        avg_y = sample['y'].mean().round(1)
        avg_th = sample['th'].mean().round(1)
        averages.append({'x': avg_x, 'y': avg_y, 'th': avg_th})

    averages_df = pd.DataFrame(averages)

    averages_df.to_csv(output_file, index=False)
    print(f"File dengan rata-rata telah disimpan ke {output_file}")


def pose_callback(msg):
    global stope

    if keyboard.read_key() == "a":
        print("berhenti mengambil trajectory")
        stope = 1

    if stope == 0:
        x = msg.pose.position.x
        y = msg.pose.position.y
        # th = msg.pose.orientation.z
        orientation_q = msg.pose.orientation
        
        (roll, pitch, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        yaw_deg = yaw * 180.0 / 3.141592653589793

        print("x: ", x)
        print("y: ", y)
        print("th: ", yaw_deg)
        with open('trajectory5.csv', mode='a') as file:
            writer = csv.writer(file)
            writer.writerow([x, y, yaw_deg])

    if stope == 1:
        stope+=1
        process_csv_with_averaging()

def listener():
    global stope

    if stope == 0:
        rospy.init_node('slam_to_csv', anonymous=True)
        
        with open('trajectory5.csv', mode='w') as file:
            writer = csv.writer(file)
            writer.writerow(['x', 'y', 'th']) 

        rospy.Subscriber('/slam_out_pose', PoseStamped, pose_callback)

        rospy.spin()


if __name__ == '__main__':
    listener()
