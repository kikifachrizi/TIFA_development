#!/usr/bin/env python3
import math
import time
import rospy
import serial
import threading
import numpy as np
from datetime import datetime

from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray

# === FIREBASE IMPORT ===
import firebase_admin
from firebase_admin import credentials, db

# ======== GLOBAL VARIABEL ========
ser2_latest = ""                # Serial buffer for both nav+firebase
current_setpoint_index = 0
processing_setpoint = False
on_setpoints2 = False
waiting_confirmation = False

# Serial: pastikan device benar!
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)     # ke Teensy navigasi
ser2 = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)    # ke ESP32/Firebase/feedback

# Setpoints
setpoints = [
    {"x": 3.0, "y": 0.0, "th": 0.0},
]
setpoints2 = [
    {"x": 0.0, "y": 0.0, "th": 0.0},
]

tol = {"vmaxtrans" : 0.2 , "vmaxrot" : 8.5 , "transtol" : 0.09 , "rottol" : 9.0} #m/s

kp = {"x" : 0.35 ,"y" : 0.35 ,"th":0.085}
ki = {"x" : 0.0035 ,"y" : 0.0015 ,"th":0.00004}
kd = {"x" : 0.0,"y" : 0.0 ,"th":0.0002}

final_output = {"x":0,"y":0,"th":0}
output = {"x":0,"y":0,"th":0}
pose = {"x":0,"y":0,"th":0}
error = {"x":0,"y":0,"th":0}
prev_err = {"x":0,"y":0,"th":0}
p = {"x":0,"y":0,"th":0}
i = {"x":0,"y":0,"th":0}
d = {"x":0,"y":0,"th":0}
twist_msg = Twist()

# === SERIAL THREAD UNTUK BACA DATA TERUS-MENERUS ===
def serial2_reader():
    global ser2_latest
    while True:
        try:
            line = ser2.readline().decode().strip()
            if line:
                ser2_latest = line
        except Exception as e:
            print("Serial2 error:", e)

# === FIREBASE SETUP & LOOP ===
def init_firebase():
    try:
        cred = credentials.Certificate("/home/tifa/catkin_ws/src/tifa_nav/serviceAccountKey.json")
        firebase_admin.initialize_app(cred, {
            'databaseURL': 'https://ta-robot-a6c67-default-rtdb.firebaseio.com/'
        })
        print("Firebase initialized successfully")
        return True
    except Exception as e:
        print(f"Firebase init error: {e}")
        return False

def firebase_update_loop():
    while True:
        try:
            line = ser2_latest
            if not line:  # No data yet
                time.sleep(0.2)
                continue
            values = line.split(';')
            ref = db.reference("/status")

            # --- SAFE CHECKS
            if len(values) < 4:
                print("Firebase: Serial data incomplete:", line)
                time.sleep(1)
                continue

            if(values[3] == '1'):
                state_door = "TERTUTUP"
            else:
                state_door = "TERBUKA"

            if(values[1] == '1'):
                state_food = "TIDAK ADA"
            else:
                state_food = "ADA"

            if(values[2] == '0'):
                state_tray = "MASUK"
            else:
                state_tray = "KELUAR"

            ref.update({
                "makanan": state_food,
                "pintu": state_door,
                "tray": state_tray,
                "last_updated": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            })

            print("✅ Data sent to Firebase at", datetime.now().strftime("%H:%M:%S"))
            time.sleep(1)
        except Exception as e:
            print(f"❌ Error updating Firebase: {e}")
            time.sleep(5)

# === NAVIGATION CONTROLLER ===
prev_time = time.time()
def controller(setpoint__):
    global prev_time,final_output
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

        output[ii] = p[ii] + i[ii] + d[ii]
        final_output[ii] = output[ii]

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

    twist_msg.linear.x = final_output["x"]
    twist_msg.linear.y = final_output["y"]
    twist_msg.angular.z = final_output["th"]

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

def callback(msg):
    global current_setpoint_index, processing_setpoint, ser2_latest, setpoints, setpoints2
    global on_setpoints2, waiting_confirmation

    x = msg.pose.position.x
    y = msg.pose.position.y
    orientation_q = msg.pose.orientation

    # print("Dari serial (latest):", ser2_latest)
    (roll, pitch, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    yaw_deg = yaw * 180.0 / 3.141592653589793

    pose["x"] = x
    pose["y"] = -1*y
    pose["th"] = -1*yaw_deg

    if waiting_confirmation and not on_setpoints2:
        twist_msg.linear.x = 0
        twist_msg.linear.y = 0
        twist_msg.angular.z = 0

        values = ser2_latest.split(';') if ser2_latest else []
        if values and values[-1] == '1':
            rospy.loginfo("Menerima konfirmasi '1' dari Teensy! Jalanin ulang dengan setpoints2.")
            setpoints = setpoints2
            current_setpoint_index = 0
            processing_setpoint = False
            on_setpoints2 = True
            waiting_confirmation = False

    elif current_setpoint_index < len(setpoints):
        if not processing_setpoint:
            rospy.loginfo("Memulai setpoint %d , %.2f , %.2f , %.2f ,",
                current_setpoint_index, setpoints[current_setpoint_index]["x"],
                setpoints[current_setpoint_index]["y"],
                setpoints[current_setpoint_index]["th"])
            processing_setpoint = True

        setpoint_reached = controller(setpoints[current_setpoint_index])

        if setpoint_reached:
            rospy.loginfo("Setpoint %d tercapai", current_setpoint_index)
            current_setpoint_index += 1
            processing_setpoint = False

    elif current_setpoint_index >= len(setpoints) and not on_setpoints2 and not waiting_confirmation:
        rospy.loginfo("Semua setpoints telah tercapai, nunggu konfirmasi dari Teensy '1'...")
        ser2.write(b'1\n')
        twist_msg.linear.x = 0
        twist_msg.linear.y = 0
        twist_msg.angular.z = 0
        waiting_confirmation = True

    elif current_setpoint_index >= len(setpoints) and on_setpoints2:
        twist_msg.linear.x = 0
        twist_msg.linear.y = 0
        twist_msg.angular.z = 0

    message = f"{round(twist_msg.linear.x, 2)} {round(twist_msg.linear.y,2)*-1} {round(twist_msg.angular.z,2)};\n"
    ser.write(message.encode('utf-8'))

# ===== MAIN RUNNER =====
if __name__ == '__main__':
    # === INIT FIREBASE ===
    if not init_firebase():
        exit(1)
    threading.Thread(target=firebase_update_loop, daemon=True).start()

    # === INIT ROS & SERIAL ===
    rospy.init_node('tifa_navigation', anonymous=True)
    threading.Thread(target=serial2_reader, daemon=True).start()

    rospy.Subscriber('/slam_out_pose', PoseStamped, callback)
    rospy.Subscriber('/new_set', Float32MultiArray, update_setpoint)
    rospy.spin()
