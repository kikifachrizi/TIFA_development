#!/usr/bin/env python3

import math 
import time
import rospy
import serial
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from simple_pid import PID
import numpy as np
import csv
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Path


current_setpoint_index = 0
processing_setpoint = False

stuck_threshold = 0.05
escape_mode = False   
sensor_angles = np.radians([-15,-30,-60,-75,-105,-120,-150,-165,-195,-210,-240,-255,-285,-300,-330,-315])
num_sensors = len(sensor_angles)

ser = serial.Serial('/dev/ttyUSB1', 57600, timeout=1)  
# bikin serial 1 lagi baca dari ESP32 database

setpoints = []

tol = {"vmaxtrans" : 0.8 , "vmaxrot" : 8.5 , "transtol" : 0.09 , "rottol" : 9.0} #m/s

kp = {"x" : 0.75 ,"y" : 0.75 ,"th":0.085}
ki = {"x" : 0.0035 ,"y" : 0.0035 ,"th":0.00004}
kd = {"x" : 0.0,"y" : 0.0 ,"th":0.0002}

resultant_force = {"x":0,"y":0,"th":0} 
escape_forces = {"x":0,"y":0,"th":0}
final_output = {"x":0,"y":0,"th":0}
repulsive_force = {}
output = {"x":0,"y":0,"th":0}
pose = {"x":0,"y":0,"th":0}
error = {"x":0,"y":0,"th":0}
prev_err = {"x":0,"y":0,"th":0}
p = {"x":0,"y":0,"th":0}
i = {"x":0,"y":0,"th":0}
d = {"x":0,"y":0,"th":0}
twist_msg = Twist()

prev_time = time.time()
distances = np.zeros(num_sensors)

def controller(setpoint__):
    global prev_time,distances,final_output,resultant_force
    current_time = time.time()
    dt = current_time - prev_time  
    prev_time = current_time

    for ii in setpoint__:
        error[ii] = setpoint__[ii] - pose[ii]

        if ii == "th":
            if error[ii] > 180:
                setpoint__[ii] -= 360
                error[ii] = setpoint__[ii] - pose[ii]
            elif error[ii] < -180:
                setpoint__[ii] += 360
                error[ii] = setpoint__[ii] - pose[ii]

        if ii == "th":
            p[ii] = kp[ii] * error[ii]
            i[ii] += ki[ii] * error[ii] 
            d[ii] = kd[ii] * (error[ii] - prev_err[ii]) 
        else:
            if error[ii] <= 1.0:
                p[ii] = kp[ii] * error[ii]
                i[ii] += ki[ii] * error[ii] 
                d[ii] = kd[ii] * (error[ii] - prev_err[ii]) 
            else:
                p[ii] = 2.5 * (kp[ii] * error[ii]) / error[ii]
                i[ii] += 2.5 * (ki[ii] * error[ii]) / error[ii] 
                d[ii] = 2.5 * (kd[ii] * (error[ii] - prev_err[ii])) / error[ii] 


        prev_err[ii] = error[ii]

        if ii == "th":
            if i[ii] > tol["vmaxrot"] :
                i[ii] = tol["vmaxrot"]
            elif i[ii] < -tol["vmaxrot"] :
                i[ii] = -tol["vmaxrot"]
        else:
            if i[ii] > tol["vmaxtrans"] :
                i[ii] = tol["vmaxtrans"]
            elif i[ii] < -tol["vmaxtrans"] :
                i[ii] = -tol["vmaxtrans"]


        if final_output[ii] < stuck_threshold:
            escape_mode = True
            escape_direction = generate_escape_direction()[0]  
            escape_force = {"x": escape_direction["x"] * 0.15, "y": escape_direction["y"] * 0.15, "th": 0.0}
        else:
            escape_mode = False
            escape_force = {"x": 0.0, "y": 0.0, "th": 0.0}


        repulsive_force = calculate_repulsive_force(distances, 0.03, sensor_angles)

        resultant_force[ii] = repulsive_force[ii] + escape_forces[ii]

        output[ii] = p[ii] + i[ii] + d[ii]
        final_output[ii] = 0.8 * output[ii] + 0.2 * resultant_force[ii]

        if ii == "th":
            if final_output[ii] > tol["vmaxrot"] :
                final_output[ii] = tol["vmaxrot"]
            elif final_output[ii] < -tol["vmaxrot"] :
                final_output[ii] = -tol["vmaxrot"]
        else:
            if final_output[ii] > tol["vmaxtrans"] :
                final_output[ii] = tol["vmaxtrans"]
            elif final_output[ii] < -tol["vmaxtrans"] :
                final_output[ii] = -tol["vmaxtrans"]

    # last output yang dikirim ke teensy
    twist_msg.linear.x = final_output["x"]
    twist_msg.linear.y = final_output["y"]
    twist_msg.angular.z = final_output["th"]
    # last output yang dikirim ke teensy
    
    if (abs(pose["x"] - setpoint__["x"]) < tol["transtol"]
        and abs(pose["y"] - setpoint__["y"]) < tol["transtol"]
        and abs(pose["th"] - setpoint__["th"]) < tol["rottol"]):

        return 1
    
    return 0



def path_callback(msg):
    global setpoints, current_setpoint_index, processing_setpoint

    setpoints = []
    for pose_stamped in msg.poses:
        x = pose_stamped.pose.position.x
        y = -pose_stamped.pose.position.y  # tergantung orientasi RViz vs dunia nyata
        orientation_q = pose_stamped.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        yaw_deg = -yaw * 180.0 / math.pi
        setpoints.append({"x": x, "y": y, "th": yaw_deg})

    current_setpoint_index = 0
    processing_setpoint = False

    rospy.loginfo("Path baru diterima. Jumlah setpoint: %d", len(setpoints))



# start ultrasonic navigation
def calculate_repulsive_force(distances, k_rep, sensor_angles):
    force = np.array([0.0, 0.0])  # Vektor gaya, komponen x dan y
    for i, d in enumerate(distances):
        d = distances[i]
        if d < 0.10 and d > 0:  # Threshold jarak rintangan
            magnitude = k_rep * (1/d - 1/0.3) * (1/d**2)
            angle = sensor_angles[i]
            force += magnitude * np.array([-np.cos(angle), -np.sin(angle)])
    
    return {"x": force[0], "y": force[1]*-1, "th": 0}


def generate_escape_direction(num_directions=1):
    escape_directions = []
    for _ in range(num_directions):
        direction = np.random.uniform(-1, 1, size=2)  # Arah acak dalam rentang -1 hingga 1
        escape_directions.append({"x": direction[0], "y": direction[1], "th": 0})
    return escape_directions

# end ultrasonic navigation


def callback(msg):
    global current_setpoint_index, processing_setpoint
    x = msg.pose.position.x 
    y = msg.pose.position.y 
    orientation_q = msg.pose.orientation
    
    (roll, pitch, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    yaw_deg = yaw * 180.0 / 3.141592653589793 

    pose["x"] = x
    pose["y"] = -1*y
    pose["th"] = -1*yaw_deg

    if current_setpoint_index < len(setpoints):
        if not processing_setpoint:
            rospy.loginfo("Memulai setpoint %d , %.2f , %.2f , %.2f ,", current_setpoint_index,setpoints[current_setpoint_index]["x"],setpoints[current_setpoint_index]["y"],setpoints[current_setpoint_index]["th"])
            processing_setpoint = True  

        setpoint_reached = controller(setpoints[current_setpoint_index])

        if setpoint_reached:
            rospy.loginfo("Setpoint %d tercapai", current_setpoint_index)
            current_setpoint_index += 8 #ini nanti diganti update dari ESP32 , bukan += 1
            processing_setpoint = False  

    if current_setpoint_index >= len(setpoints):
        rospy.loginfo("Semua setpoints telah tercapai")
        twist_msg.linear.x = 0
        twist_msg.linear.y = 0
        twist_msg.angular.z = 0


    # start transfer data ke ros msgs dan Teensy 4.1
    message = f"{round(twist_msg.linear.x, 2)} {round(twist_msg.linear.y,2)*-1} {round(twist_msg.angular.z,2)};\n"
    ser.write(message.encode('utf-8'))

    # end transfer data ke ros msgs dan Teensy 4.1

def ultrasonic_callback(msg):
    global distances
    if len(msg.data) != num_sensors:
        rospy.logwarn("Jumlah sensor tidak sesuai! Harus 16 data.")
        return

    for i in range(num_sensors):
        distances = np.array(msg.data) / 100.0  # Konversi dari cm ke meter



if __name__ == '__main__':
    rospy.init_node('tifa_navigation', anonymous=True)
    
    # pid_analisys_pub = rospy.Publisher("pid_analisys", String, queue_size=10)
    rospy.Subscriber('/slam_out_pose', PoseStamped, callback)
    # rospy.Subscriber('/new_set', Float32MultiArray, update_setpoint)
    rospy.Subscriber('/move_base/NavfnROS/plan', Path, path_callback)
    rospy.Subscriber('/ultrasonic_data', Float32MultiArray, ultrasonic_callback)
    rospy.spin()


