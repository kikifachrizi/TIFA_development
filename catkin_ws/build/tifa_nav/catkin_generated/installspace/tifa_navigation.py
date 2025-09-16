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

current_setpoint_index = 0
processing_setpoint = False

stuck_threshold = 0.05
escape_mode = False   
sensor_angles = np.radians([-15,-30,-60,-75,-105,-120,-150,-165,-195,-210,-240,-255,-285,-300,-330,-315])
num_sensors = len(sensor_angles)

ser = serial.Serial('/dev/ttyUSB1' , 57600, timeout=1)  
# esp1 = serial.Serial('/dev/ttyUSB3', 9600 , timeout=1)

setpoints = [
    # {"x": 0.2, "y": 0.0, "th": 0.0},
    {"x": 0.0, "y": 0.0, "th": 0.0},
    # {"x": 1.5, "y": 0.0, "th": 0.0},
    # {"x": 1.5, "y": -1.2, "th": 0.0},
    # {"x": 5.0, "y": -1.2, "th": 0.0},

    # {"x": 5.0, "y": -1.2, "th": 0.0},
    # {"x": 1.5, "y": -1.2, "th": 0.0},
    # {"x": 1.5, "y": 0.0, "th": 0.0},
    # {"x": 0.0, "y": 0.0, "th": 0.0},
    # # {"x": 1.2, "y": -0.4, "th": 0.0},
    # {"x": 1.4, "y": -0.4, "th": 0.0},
    # {"x": 1.6, "y": -0.4, "th": 0.0},
    # {"x": 1.8, "y": -0.4, "th": 0.0},
    # {"x": 2.2, "y": -0.4, "th": 0.0},
    # {"x": 3.0, "y": -1.0, "th": 0.0},
    # {"x": 1.0, "y": 0.0, "th": 0.0},
    # {"x": 6.3, "y": -2.0, "th": 0.0},
    # {"x": 7.0, "y": -2.0, "th": 0.0},
]

tol = {"vmaxtrans" : 0.6 , "vmaxrot" : 8.5 , "transtol" : 0.09 , "rottol" : 9.0} #m/s   

kp = {"x" : 0.35 ,"y" : 0.35 ,"th":0.085}
ki = {"x" : 0.0035 ,"y" : 0.0015 ,"th":0.00004}
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
            escape_force = {"x": escape_direction["x"] * 0.008, "y": escape_direction["y"] * 0.008, "th": 0.0}
        else:
            escape_mode = False
            escape_force = {"x": 0.0, "y": 0.0, "th": 0.0}


        repulsive_force = calculate_repulsive_force(distances, 0.005, sensor_angles)

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



def update_setpoint(msg):
    global setpoints, current_setpoint_index, processing_setpoint

    data = msg.data
    num_points = len(data) // 3  
    setpoints = []
    for i in range(num_points):
        x = data[i * 3]
        y = data[i * 3 + 1]
        th = data[i * 3 + 2]
        setpoints.append({"x": x, "y": y, "th": th})
    
    current_setpoint_index = 0  
    processing_setpoint = False  

    rospy.loginfo("Setpoints received:")
    for idx, sp in enumerate(setpoints):
        rospy.loginfo("  Setpoint %d: x=%.2f, y=%.2f, th=%.2f", idx + 1, sp["x"], sp["y"], sp["th"])



# start ultrasonic navigation
def calculate_repulsive_force(distances, k_rep, sensor_angles):
    force = np.array([0.0, 0.0])  # Vektor gaya, komponen x dan y
    for i, d in enumerate(distances):
        d = distances[i]
        if d < 0.20 and d > 0:  # Threshold jarak rintangan
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
            current_setpoint_index += 1
            processing_setpoint = False  

    if current_setpoint_index >= len(setpoints):
        rospy.loginfo("Semua setpoints telah tercapai")
        twist_msg.linear.x = 0
        twist_msg.linear.y = 0
        twist_msg.angular.z = 0


    # start transfer data ke ros msgs dan Teensy 4.1
    message = f"{round(twist_msg.linear.x, 2)} {round(twist_msg.linear.y,2)*-1} {round(twist_msg.angular.z,2)};\n"

    # rospy.loginfo("err_x: %.2f | act x: %.2f | err_y: %.2f | act y: %.2f | err_th: %.2f | act th: %.2f |", error["x"],pose["x"],error["y"],pose["y"],error["th"],pose["th"],)
    # pid_analisys = (
    #     "%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f" % (
    #         round(twist_msg.linear.x, 2),round(twist_msg.linear.y, 2),round(twist_msg.angular.z, 2),
    #         pose["x"],pose["y"],pose["th"],
    #         error["x"],error["y"],error["th"],
    #     )
    # ) 
    # pid_analisys_pub.publish(pid_analisys) 
    ser.write(message.encode('utf-8'))
    # end transfer data ke ros msgs dan Teensy 4.1

def ultrasonic_callback(msg):
    global distances
    if len(msg.data) != num_sensors:
        rospy.logwarn("Jumlah sensor tidak sesuai! Harus 16 data.")
        return

    for i in range(num_sensors):
        distances = np.array(msg.data) / 100.0  # Konversi dari cm ke meter
        # print(distances)



if __name__ == '__main__':
    rospy.init_node('tifa_navigation', anonymous=True)
    
    # pid_analisys_pub = rospy.Publisher("pid_analisys", String, queue_size=10)

    rospy.Subscriber('/slam_out_pose', PoseStamped, callback)
    rospy.Subscriber('/new_set', Float32MultiArray, update_setpoint)
    rospy.Subscriber('/ultrasonic_data', Float32MultiArray, ultrasonic_callback)
    rospy.spin()


