#!/usr/bin/env python3
import rospy
import numpy as np
import serial
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped


ser = serial.Serial('/dev/ttyUSB1', 57600, timeout=1)

setpoints = [
    {"x": 2.0, "y": 0.0, "th": 0.0},
    {"x": 0.0, "y": 0.0, "th": 0.0},
]

current_target_idx = 0  

# Inisialisasi Parameter Robot
robot_state = np.array([0.0, 0.0, 0.0])  # [Vx, Vy, Vth]
robot_pos = np.array([0.0, 0.0])  # Posisi robot akan diambil dari /slam_out_pose
robot_theta = 0.0                 # Orientasi robot juga diambil dari /slam_out_pose
dt = 0.1                           # Waktu per iterasi
max_speed = 0.3                    # Kecepatan maksimum robot
stuck_threshold = 0.05              # Threshold jika robot macet
escape_mode = False                 # Apakah robot dalam Escape Mode

# Sensor Ultrasonik (12 buah dengan sudut yang diberikan)
sensor_angles = np.radians([0, -30, -60, -90, -120, -150, -180, -210, -240, -270, -300, -330])

# Callback untuk mendapatkan posisi robot dari /slam_out_pose
def slam_pose_callback(msg):
    global robot_pos, robot_theta
    robot_pos = np.array([msg.pose.position.x, msg.pose.position.y])
    
    # Ambil orientasi dari quaternion ke yaw (theta)
    q = msg.pose.orientation
    robot_theta = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))

# Callback untuk membaca data dari 12 sensor ultrasonik
def ultrasonic_callback(msg):
    global robot_state, escape_mode, current_target_idx

    if len(msg.data) != 12:
        rospy.logwarn("Jumlah sensor tidak sesuai! Harus 12 data.")
        return

    distances = np.array(msg.data) / 100.0  # Konversi dari cm ke meter

    # Ambil target posisi saat ini dari setpoints
    if current_target_idx < len(setpoints):
        target = setpoints[current_target_idx]
        target_pos = np.array([target["x"], target["y"]])
        target_theta = target["th"]
    else:
        rospy.loginfo("Semua setpoints telah tercapai!")
        send_serial_command(0.0, 0.0, 0.0)  # Hentikan robot
        return

    # Hitung Gaya Attractive (menuju target)
    attractive_force = calculate_attractive_force(robot_pos, target_pos, 1.2)

    # Hitung Gaya Repulsive (menjauhi rintangan)
    repulsive_force = calculate_repulsive_force(distances, 2.5)

    # Escape Mode jika robot macet
    if np.linalg.norm(robot_state[:2]) < stuck_threshold:
        escape_mode = True
        escape_direction = generate_escape_direction()
        escape_force = np.array([escape_direction[0] * 1.5, escape_direction[1] * 1.5, 0.0])
    else:
        escape_mode = False
        escape_force = np.array([0.0, 0.0, 0.0])

    # Kombinasi semua gaya
    resultant_force = np.array([*attractive_force, 0.0]) + np.array([*repulsive_force, 0.0]) + escape_force

    # Update kecepatan robot (Vx, Vy, Vth)
    robot_state = 0.7 * robot_state + 0.3 * resultant_force

    # Batasi kecepatan maksimal
    if np.linalg.norm(robot_state[:2]) > max_speed:
        robot_state[:2] = (robot_state[:2] / np.linalg.norm(robot_state[:2])) * max_speed

    # Jika robot sudah dekat dengan target, pindah ke target berikutnya
    if np.linalg.norm(robot_pos - target_pos) < 0.2 and abs(robot_theta - target_theta) < 0.1:
        rospy.loginfo(f"Setpoint {current_target_idx} tercapai! Pindah ke berikutnya.")
        current_target_idx += 1

    # Kirim perintah ke serial /dev/ttyUSB1
    send_serial_command(robot_state[0], robot_state[1], robot_state[2])

# Fungsi untuk menghitung gaya tarik ke target
def calculate_attractive_force(robot_pos, target_pos, k_att):
    direction = target_pos - robot_pos
    distance = np.linalg.norm(direction)
    if distance <= 2:
        return k_att * direction
    else:
        return 2 * k_att * direction / distance

# Fungsi untuk menghitung gaya tolak dari rintangan
def calculate_repulsive_force(distances, k_rep):
    force = np.array([0.0, 0.0])
    for i, d in enumerate(distances):
        if d < 0.5 and d > 0:  # Threshold jarak rintangan
            magnitude = k_rep * (1/d - 1/0.5) * (1/d**2)
            angle = sensor_angles[i]
            force += magnitude * np.array([-np.cos(angle), -np.sin(angle)])
    return force

# Fungsi untuk Escape Mode
def generate_escape_direction():
    return np.random.uniform(-1, 1, size=2)  # Random arah keluar

# Fungsi untuk mengirim data ke Serial /dev/ttyUSB1
def send_serial_command(Vx, Vy, Vth):
    command = f"{Vx:.2f} {Vy:.2f} {Vth:.2f} {0.0} {0.0} {0.0}\n"
    ser.write(command.encode('utf-8'))
    print(robot_pos[0], robot_pos[1], robot_theta)
    # rospy.loginfo(f"Serial Sent: {command.strip()}")

# Main ROS Node
if __name__ == '__main__':
    rospy.init_node('ultrasonic_navigation')

    # Subscriber untuk mendapatkan posisi robot dari /slam_out_pose
    rospy.Subscriber('/slam_out_pose', PoseStamped, slam_pose_callback)

    # Subscriber untuk sensor ultrasonik
    rospy.Subscriber('/ultrasonic_data', Float32MultiArray, ultrasonic_callback)

    rospy.spin()
