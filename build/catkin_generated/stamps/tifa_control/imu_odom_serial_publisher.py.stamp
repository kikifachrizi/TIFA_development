#!/usr/bin/env python3

import rospy
import serial
import math
import tf
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import threading

ser2 = serial.Serial('/dev/ttyUSB1', 19200, timeout=1)  #cp210x

data =""
pub = None

def read_serial_data():
    global data
    while not rospy.is_shutdown():
        if ser2.in_waiting > 0:
            data = ser2.readline().decode('utf-8', errors='ignore')  # Ignore invalid chars
            # print(f"Received data: {data}")

            # Pastikan data tidak kosong setelah decoding
            if data:
                parts = data.strip().split(";")
                if len(parts) >= 11:  # Pastikan ada cukup data untuk diproses
                    accel = [float(parts[0]), float(parts[1]), float(parts[2])]  # Akselerasi
                    gyro = [float(parts[3]), float(parts[4]), float(parts[5])]  # Gyroscope
                    angle = [float(parts[6]), float(parts[7]), float(parts[8])]  # Sudut
                    pos = [float(parts[9]), float(parts[10])]  # Posisi X dan Y
                    
                else:
                    rospy.logwarn(f"Data received is incomplete: {data}")
                    accel = [0.0, 0.0, 0.0]
                    gyro = [0.0, 0.0, 0.0]
                    angle = [0.0, 0.0, 0.0]
                    pos = [0.0, 0.0]


            # Publish IMU
            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = "imu_link"

            imu_msg.linear_acceleration.x = accel[0]
            imu_msg.linear_acceleration.y = accel[1]
            imu_msg.linear_acceleration.z = accel[2]

            imu_msg.angular_velocity.x = math.radians(gyro[0])
            imu_msg.angular_velocity.y = math.radians(gyro[1])
            imu_msg.angular_velocity.z = math.radians(gyro[2])

            quat = tf.transformations.quaternion_from_euler(
                math.radians(angle[0]),  # roll
                math.radians(angle[1]),  # pitch
                math.radians(angle[2])   # yaw
            )
            imu_msg.orientation = Quaternion(*quat)
            imu_msg.orientation_covariance = [0.01, 0, 0,
                                  0, 0.01, 0,
                                  0, 0, 0.01]
            imu_msg.angular_velocity_covariance = [0.02, 0, 0,
                                       0, 0.02, 0,
                                       0, 0, 0.02]
            imu_msg.linear_acceleration_covariance = [0.1, 0, 0,
                                          0, 0.1, 0,
                                          0, 0, 0.1]

            imu_pub.publish(imu_msg)

            # print(pos[0])
            # Publish Odometry
            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = "map"
            odom_msg.child_frame_id = "base_link"

            odom_msg.pose.pose.position.x = pos[0]
            odom_msg.pose.pose.position.y = pos[1]
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation = Quaternion(*quat)

            odom_msg.twist.twist.linear.x = 0
            odom_msg.twist.twist.linear.y = 0
            odom_msg.twist.twist.angular.z = 0
            odom_msg.pose.covariance = [0.1 if i % 7 == 0 else 0 for i in range(36)]
            odom_msg.twist.covariance = [0.2 if i % 7 == 0 else 0 for i in range(36)]

            odom_pub.publish(odom_msg)

            # Broadcast TF
            br.sendTransform(
                (pos[0], 
                pos[1], 
                0),
                quat,
                rospy.Time.now(),
                "base_link",
                "odom"
            )



def main():
    global imu_pub, odom_pub, br
    rospy.init_node('imu_odom_serial_pub', anonymous=True)
    # pub = rospy.Publisher("/raw_data_pid", String, queue_size=10)
    imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    br = tf.TransformBroadcaster()

    thread = threading.Thread(target=read_serial_data, daemon=True)
    thread.start()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
