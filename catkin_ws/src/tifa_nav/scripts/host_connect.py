#!/usr/bin/env python3
import rospy
import serial
import requests

# === KONFIGURASI ===
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
ROBOT_ID = 3  # ganti sesuai kebutuhan
API_URL = f"https://be.tifa-app.my.id/api/robotData/{ROBOT_ID}"
HEADERS = {"Content-Type": "application/json"}  # kalau perlu token auth, tambahkan di sini

def kirim_battery(battery_val, robot_id):
    url = f"https://be.tifa-app.my.id/api/robotData/{robot_id}"
    payload = {
        "battery_level": battery_val,
        "battery_performance": battery_val
    }
    try:
        response = requests.put(url, json=payload, headers=HEADERS, timeout=5)
        if response.status_code in [200, 201]:
            rospy.loginfo(f"[UPDATE] Battery level={battery_val} id={robot_id} OK: {response.text}")
        else:
            rospy.logwarn(f"[UPDATE] Gagal update battery id={robot_id}, status={response.status_code}: {response.text}")
    except Exception as e:
        rospy.logerr(f"[ERROR] Gagal kirim data ke server: {e}")

if __name__ == "__main__":
    rospy.init_node("battery_update_node")
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=2)
    rate = rospy.Rate(0.2)  # tiap 5 detik

    rospy.loginfo("Node aktif, baca data serial dan update battery ke API...")

    while not rospy.is_shutdown():
        try:
            line = ser.readline().decode().strip()
            if line:
                rospy.loginfo(f"[SERIAL] Data masuk: {line}")
                # contoh: "0;0;95"
                try:
                    battery_val = int(line.split(";")[-1])
                    kirim_battery(battery_val, ROBOT_ID)
                except Exception as e:
                    rospy.logwarn(f"[PARSE] Data gagal diparse: {e}")
        except Exception as e:
            rospy.logerr(f"[SERIAL] Error: {e}")
        rate.sleep()
