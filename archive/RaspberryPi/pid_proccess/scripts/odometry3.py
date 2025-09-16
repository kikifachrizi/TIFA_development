#!/usr/bin/env python3

import math
import time
import rospy
import serial
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from pandas import read_csv
from simple_pid import PID

# Sesuaikan dengan port dan baudrate Anda
ser = serial.Serial('/dev/ttyUSB1', 57600, timeout=1)

# Membaca data dari CSV yang berisi koordinat x, y
data = read_csv("~/catkin_ws/src/pid_proccess/scripts/trajectorycoba.csv")
setpoints = [{"x": x, "y": y, "th": 0.0} for x, y in zip(data['x'], data['y'])]

# Parameter toleransi dan PID
tol = {"vmaxtrans": 2.0, "vmaxrot": 8.0, "transtol": 0.2, "rottol": 5.0}  # cm/s
kp = {"x": 4.07, "y": 4.05, "th": 0.005}
ki = {"x": 0.02, "y": 0.01, "th": 0.00001}
kd = {"x": 0.0001, "y": 0.0001, "th": 0.00}

pose = {"x": 0, "y": 0, "th": 0}  # posisi robot
error = {"x": 0, "y": 0, "th": 0}  # error robot
prev_err = {"x": 0, "y": 0, "th": 0}  # previous error untuk PID
p = {"x": 0, "y": 0, "th": 0}  # PID terms
i = {"x": 0, "y": 0, "th": 0}
d = {"x": 0, "y": 0, "th": 0}
output = {"x": 0, "y": 0, "th": 0}

twist_msg = Twist()

# Membaca posisi robot dari topik /slam_out_pose
def callback(msg):
    global pose, prev_err, output, twist_msg
    # Membaca posisi robot (x, y) dan orientasi (th)
    x = msg.pose.position.x
    y = msg.pose.position.y
    orientation_q = msg.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    yaw_deg = yaw * 180.0 / 3.141592653589793

    pose["x"] = x
    pose["y"] = -1 * y  # Asumsi Y perlu dibalik sesuai kebutuhan
    pose["th"] = -1 * yaw_deg  # Asumsi orientasi perlu dibalik

    # Memulai pengendalian PID
    controller()

# Pengendalian PID
def controller():
    global prev_err, output, twist_msg

    # Proses setpoint saat ini
    setpoint = setpoints[0]  # Setpoint pertama yang diambil
    error["x"] = setpoint["x"] - pose["x"]
    error["y"] = setpoint["y"] - pose["y"]
    error["th"] = setpoint["th"] - pose["th"]

    # Proses PID untuk x, y, th
    for ii in ["x", "y", "th"]:
        p[ii] = kp[ii] * error[ii]
        i[ii] += ki[ii] * error[ii]
        d[ii] = kd[ii] * (error[ii] - prev_err[ii])

        prev_err[ii] = error[ii]

        # Pembatasan output PID
        output[ii] = p[ii] + i[ii] + d[ii]
        if ii == "th":
            output[ii] = max(min(output[ii], tol["vmaxrot"]), -tol["vmaxrot"])
        else:
            output[ii] = max(min(output[ii], tol["vmaxtrans"]), -tol["vmaxtrans"])

    # Mengirim perintah ke robot
    twist_msg.linear.x = output["x"]
    twist_msg.linear.y = output["y"]
    twist_msg.linear.z = output["th"]

    # Kirimkan pesan kontrol ke robot melalui serial
    message = f"{round(twist_msg.linear.x, 2)};{round(twist_msg.linear.y, 2)};{round(twist_msg.linear.z, 2)};\n"
    ser.write(message.encode('utf-8'))  # Mengirim data ke STM32 melalui serial

    # Mengecek apakah setpoint tercapai
    if abs(error["x"]) < tol["transtol"] and abs(error["y"]) < tol["transtol"] and abs(error["th"]) < tol["rottol"]:
        rospy.loginfo("Setpoint tercapai: %s", setpoint)
        setpoints.pop(0)  # Menghapus setpoint yang sudah tercapai

    # Kirimkan pesan kontrol ke robot
    rospy.Publisher('/cmd_vel', Twist, queue_size=10).publish(twist_msg)

def listener():
    rospy.init_node('pid_controller', anonymous=True)
    rospy.Subscriber('/slam_out_pose', PoseStamped, callback)  # Posisi robot
    rospy.spin()

if __name__ == '__main__':
    listener()
