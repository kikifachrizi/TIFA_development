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
from pandas import *

ser = serial.Serial('/dev/ttyUSB1', 57600, timeout=1)  # Sesuaikan dengan port dan baudrate Anda

data = read_csv("~/catkin_ws/src/pid_proccess/scripts/telcaf.csv")

setpoints = []

x = data['x'].tolist()
y = data['y'].tolist()

for idx in range(len(x)):
    cord = {"x": x[idx], "y": y[idx], "th": 0.0}
    setpoints.append(cord)

# setpoints = [
#     {"x": 0.1, "y": 0.0, "th": 0.0},  # Home
#     # {"x": 0.90, "y": 0.0, "th": 0.0},  # Setpoint 1
#     # {"x": 0.90, "y": -0.34, "th": 0.0},   # Setpoint 2
#     # {"x": 2.0, "y": -0.34, "th": 0.0},
#     # {"x": 10.11, "y": -2.7, "th": 0.0},
#     # {"x": 0.0, "y": 0.0, "th": 0.0}, #cm??
#     # {"x": 0.0, "y": 0.0, "th": 90.0}
# ]

tol = {"vmaxtrans" : 2.0 , "vmaxrot" : 2.0 , "transtol" : 0.5 , "rottol" : 2.0}

kp = {"x" : 5.07 ,"y" : 4.05 ,"th":0.005}
ki = {"x" : 0.02 ,"y" : 0.01 ,"th":0.00001}
kd = {"x" : 0.0001,"y" : 0.0001 ,"th":0.00}

# tol = {"vmaxtrans": 0.2, "vmaxrot": 0.5, "transtol": 0.01, "rottol": 1.0}
# kp = {"x": 1.0, "y": 1.0, "th": 0.1}
# ki = {"x": 0.0, "y": 0.0, "th": 0.0}
# kd = {"x": 0.01, "y": 0.01, "th": 0.01}

output = {"x":0,"y":0,"th":0}
pose = {"x":0,"y":0,"th":0}
error = {"x":0,"y":0,"th":0}
prev_err = {"x":0,"y":0,"th":0}
p = {"x":0,"y":0,"th":0}
i = {"x":0,"y":0,"th":0}
d = {"x":0,"y":0,"th":0}
twist_msg = Twist()



# def controller(setpoint__):
#     for ii in setpoint__:
#         error[ii] = setpoint__[ii] - pose[ii]

#         if ii == "th":
#             if error[ii] > 180:
#                 setpoint__[ii] -= 360
#                 error[ii] = setpoint__[ii] - pose[ii]
#             elif error[ii] < -180:
#                 setpoint__[ii] += 360
#                 error[ii] = setpoint__[ii] - pose[ii]

#         # print(types(i) ," = ", error[i])

#         p[ii] = kp[ii] * error[ii]
#         i[ii] += ki[ii]*error[ii]
#         d[ii] = kd[ii]* (error[ii] - prev_err[ii])

#         prev_err[ii] = error[ii]
#         if ii == "th":
#             if i[ii] > tol["vmaxrot"]:
#                 i[ii] = tol["vmaxrot"]
#             elif i[ii] < -tol["vmaxrot"]:
#                 i[ii] = -tol["vmaxrot"]
#         else:
#             if i[ii] > tol["vmaxtrans"]:
#                 i[ii] = tol["vmaxtrans"]
#             elif i[ii] < -tol["vmaxtrans"]:
#                 i[ii] = -tol["vmaxtrans"]


#         output[ii] = p[ii] + i[ii] + d[ii]

#         if ii == "th":
#             if output[ii] > tol["vmaxrot"]:
#                 output[ii] = tol["vmaxrot"]
#             elif output[ii] < -tol["vmaxrot"]:
#                 output[ii] = -tol["vmaxrot"]
#         else:
#             if output[ii] > tol["vmaxtrans"]:
#                 output[ii] = tol["vmaxtrans"]
#             elif output[ii] < -tol["vmaxtrans"]:
#                 output[ii] = -tol["vmaxtrans"]

#     #pid end

#     twist_msg.linear.x = output["x"]
#     twist_msg.linear.y = output["y"]
#     twist_msg.linear.z = output["th"]

#     if (abs(pose["x"] - setpoint__["x"]) < tol["transtol"] and 
#         abs(pose["y"] - setpoint__["y"]) < tol["transtol"] and 
#         abs(pose["th"] - setpoint__["th"]) < tol["rottol"]):
#         return 1

#     return 0

prev_time = time.time()  # Waktu sebelumnya
def controller(setpoint__):
    global prev_time

    current_time = time.time()
    dt = current_time - prev_time  # Waktu sampling
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

        # PID calculation
        p[ii] = kp[ii] * error[ii]
        i[ii] += ki[ii] * error[ii] * dt  # Integral menggunakan dt
        d[ii] = kd[ii] * (error[ii] - prev_err[ii]) / dt  # Diferensial menggunakan dt

        prev_err[ii] = error[ii]

        # Batasan Integral
        i[ii] = max(min(i[ii], tol["vmaxtrans" if ii != "th" else "vmaxrot"]), -tol["vmaxtrans" if ii != "th" else "vmaxrot"])

        # Hasil PID
        output[ii] = p[ii] + i[ii] + d[ii]

        # Batasan Output
        output[ii] = max(min(output[ii], tol["vmaxtrans" if ii != "th" else "vmaxrot"]), -tol["vmaxtrans" if ii != "th" else "vmaxrot"])

    # Twist Message
    twist_msg.linear.x = output["x"]
    twist_msg.linear.y = output["y"]
    twist_msg.linear.z = output["th"]

    # Periksa apakah setpoint tercapai
    print("X : ", abs(pose["x"] - setpoint__["x"]) < tol["transtol"])
    print("Y: ", abs(pose["y"] - setpoint__["y"]) < tol["transtol"])
    if (abs(pose["x"] - setpoint__["x"]) < tol["transtol"] and 
        abs(pose["y"] - setpoint__["y"]) < tol["transtol"] and 
        abs(pose["th"] - setpoint__["th"]) < tol["rottol"]):
        return 1

    return 0



def update_setpoint(msg):
    global setpoints, current_setpoint_index, processing_setpoint

    data = msg.data
    num_points = len(data) // 3  # Setiap tujuan memiliki 3 elemen (x, y, th)

    # Perbarui setpoints dengan data baru
    setpoints = []
    for i in range(num_points):
        x = data[i * 3]
        y = data[i * 3 + 1]
        th = data[i * 3 + 2]
        setpoints.append({"x": x, "y": y, "th": th})
    
    # Reset status untuk memproses tujuan baru
    current_setpoint_index = 0  # Mulai dari tujuan pertama
    processing_setpoint = False  # Siap memproses setpoint baru

    # Log semua setpoints yang diterima
    rospy.loginfo("Setpoints received:")
    for idx, sp in enumerate(setpoints):
        rospy.loginfo("  Setpoint %d: x=%.2f, y=%.2f, th=%.2f", idx + 1, sp["x"], sp["y"], sp["th"])



current_setpoint_index = 0
processing_setpoint = False
def callback(msg):
    global current_setpoint_index, processing_setpoint
    x = msg.pose.position.x #current post x
    y = msg.pose.position.y #current post y
    orientation_q = msg.pose.orientation
    
    (roll, pitch, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    yaw_deg = yaw * 180.0 / 3.141592653589793 #current orientation
    
    #pid start
    pose["x"] = x
    pose["y"] = -1*y
    pose["th"] = -1*yaw_deg


    # Pastikan controller hanya memproses satu setpoint pada satu waktu
    if current_setpoint_index < len(setpoints):
        if not processing_setpoint:
            rospy.loginfo("Memulai setpoint %d", current_setpoint_index)
            # print("X : " + setpoints[1]['x'])
            processing_setpoint = True  # Tandai bahwa robot sedang memproses setpoint

        # Jalankan controller untuk setpoint saat ini
        setpoint_reached = controller(setpoints[current_setpoint_index])

        # Jika setpoint tercapai, pindah ke setpoint berikutnya
        if setpoint_reached:
            rospy.loginfo("Setpoint %d tercapai", current_setpoint_index)
            current_setpoint_index += 1
            processing_setpoint = False  # Reset status untuk setpoint berikutnya

    # Jika semua setpoints selesai, hentikan robot
    if current_setpoint_index >= len(setpoints):
        rospy.loginfo("Semua setpoints telah tercapai")
        twist_msg.linear.x = 0
        twist_msg.linear.y = 0
        twist_msg.linear.z = 0


    message = f"{round(twist_msg.linear.x, 2)};{round(twist_msg.linear.y,2)};{round(twist_msg.linear.z,2)};\n"
    # msg_debug = f"{2};{0};{0};\n"

    # rospy.loginfo("err_x: %.2f | act x: %.2f | sp_x: %.2f | output_x: %.2f", error["x"],pose["x"],setpoints[0]["x"],output["x"])
    # rospy.loginfo("err_y: %.2f | act y: %.2f | sp_y: %.2f | output_y: %.2f", error["y"],pose["y"],setpoints[0]["y"],output["y"])
    # rospy.loginfo("err_th: %.2f | act th: %.2f | sp_th: %.2f | output_th: %.2f", error["th"],pose["th"],setpoints[0]["th"],output["th"])
    rospy.loginfo("err_x: %.2f | act x: %.2f | err_y: %.2f | act y: %.2f | err_th: %.2f | act th: %.2f |", error["x"],pose["x"],error["y"],pose["y"],error["th"],pose["th"],)
    # rospy.loginfo("out_x: %.2f | out_y: %.2f | out_th: %.2f", output["x"],output["y"],output["th"])

    pid_analisys = (
        "%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f" % (
            round(twist_msg.linear.x, 2),round(twist_msg.linear.y, 2),round(twist_msg.linear.z, 2),
            pose["x"],pose["y"],pose["th"],
            error["x"],error["y"],error["th"],
        )
    ) #data yang di publish ke topics

    cmd_vel_pub.publish(twist_msg)





    pid_analisys_pub.publish(pid_analisys) #publish process


    ser.write(message.encode('utf-8'))

if __name__ == '__main__':
    rospy.init_node('odometry', anonymous=True)
    
    pid_analisys_pub = rospy.Publisher("pid_analisys", String, queue_size=10)


    rospy.Subscriber('/slam_out_pose', PoseStamped, callback)
    rospy.Subscriber('/new_set', Float32MultiArray, update_setpoint)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


    rospy.spin()