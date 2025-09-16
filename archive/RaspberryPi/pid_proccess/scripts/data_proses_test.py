#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
import math
import time
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, PointStamped, Twist
from std_msgs.msg import String

# Initialize arrays
kp = [1.0, 1.0, 1.0]
ki = [0.01, 0.01, 0.01]
kd = [0.00, 0.00, 0.00]
pose = [0, 0, 0]

# Target position in global frame
target_position = {"x": 0.5, "y": 0.0, "z": 0}
TOLERANCE = 0.05
MAX_INTEGRAL = 1.0
error = [0, 0, 0]
prev_error = [0, 0, 0]
p = [0, 0, 0]
i = [0, 0, 0]
d = [0, 0, 0]
output = [0, 0, 0]


def normalize_angle(angle):

    # First, ensure the angle is within -180 to 180
    angle = (angle + 180) % 360 - 180
    # Convert negative angles to positive equivalent
    if angle < 0:
        angle += 360
    return angle


def poseupdate_callback(msg):
    global pose, output, twist_msg

    try:
        # Prepare global position
        position_global = PointStamped()
        position_global.header.frame_id = "map"
        position_global.header.stamp = rospy.Time(0)
        position_global.point = msg.pose.position
        yaw = msg.pose.orientation
        x, y, z = euler_from_quaternion([yaw.x, yaw.y, yaw.z, yaw.w])
        z = normalize_angle(math.degrees(z))
        # Set pose from global position
        pose[0] = position_global.point.x
        pose[1] = position_global.point.y
        pose[2] = z  # Using z component of orientation

        # Set setpoint
        setpoint = [target_position["x"], target_position["y"], target_position["z"]]

        # PID control loop
        for ii in range(3):
            error[ii] = setpoint[ii] - pose[ii]

            # Handle angle wrapping for rotation
            if ii == 2:
                if error[ii] > 180:
                    setpoint[ii] -= 360
                    error[ii] = setpoint[ii] - pose[ii]
                elif error[ii] < -180:
                    setpoint[ii] += 360
                    error[ii] = setpoint[ii] - pose[ii]

            # Calculate PID components
            p[ii] = kp[ii] * error[ii]
            i[ii] += ki[ii] * error[ii]
            d[ii] = kd[ii] * (error[ii] - prev_error[ii])

            prev_error[ii] = error[ii]

            # Calculate output
            output[ii] = p[ii] + i[ii] + d[ii]

        # Prepare Twist message
        x = output[0] * math.cos(z * 0.0174533) + output[1] * math.sin(z * 0.0174533)   # Linear velocity in x-axis
        y = -output[0] * math.sin(z * 0.0174533) + output[1] * math.cos(z * 0.0174533)  # Linear velocity in y-axis
        z = -output[2]
        print(pose[0],setpoint[0],x)

        # Prepare processed data string
        processed_data = (
            "Processed PoseUpdate: global_x=%.2f, global_y=%.2f, global_z=%.2f | "
            "Control (twist): x=%.2f, y=%.2f, z=%.2f"
            % (pose[0], pose[1], pose[2], x,y,z)
        )

        # Publish processed data and twist
        processed_pub.publish(processed_data)
        #twist_pub.publish(twist_msg)

    except Exception as e:
        rospy.logwarn("Failed to process pose: %s", str(e))

# Main function
if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node("data_processor", anonymous=True)

    # TF2 Buffer and Listener for transformations
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Publishers for processed data and twist
    processed_pub = rospy.Publisher("processed_data", String, queue_size=10)
    twist_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    # Subscriber for /slam_out_pose
    rospy.Subscriber("/slam_out_pose", PoseStamped, poseupdate_callback)

    # Spin to keep node running
    rospy.spin()
