#!/usr/bin/env python3
import rospy
import requests
import json
from std_msgs.msg import Int32,String
import serial

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  

SERVER_HOST = "http://192.168.145.114/"
UPDATE_PATH = "tes_app/sensor_status.php"
GET_PATH = "tes_app/sensor_status.php"

status_to_post = 0
coordinates = ""


def update_sensor_data():
    try:
        line = ser.readline().decode().strip()
        values = line.split(';')
        
        if len(values) < 7:
            rospy.logwarn("Data tidak lengkap! Dapat %d nilai, butuh 7", len(values))
            return

        sensor_data = {
            "action": "update",
            "id": 1,
            "ir": int(values[0]),
            "tegangan_asli": float(values[1]),
            "persentase_baterai": int(values[2]),
            "limit_switch_1": int(values[3]),
            "limit_switch_2": int(values[4]),
            "limit_switch_3": int(values[5]),
            "limit_switch_4": int(values[6]),
        }
        
        rospy.loginfo("Mengirim data: %s", sensor_data)
        
        response = requests.post(
            SERVER_HOST + UPDATE_PATH,
            data=sensor_data,  # Menggunakan form-data
            timeout=5
        )

        if response.status_code == 200:
            rospy.loginfo("[UPDATE] Berhasil update: %s", response.text)
        else:
            rospy.logwarn("[UPDATE] Gagal update: %d - %s", 
                         response.status_code, response.text)
            
    except Exception as e:
        rospy.logerr("Error: %s", str(e))


def get_coordinates():
    try:
        response = requests.get(
            SERVER_HOST + GET_PATH,
            headers={'Accept': 'application/json'},
            timeout=5
        )
        
        rospy.loginfo("GET Response: %d - %s", response.status_code, response.text)
        
        if response.status_code == 200:
            try:
                data = response.json()  # Langsung parse JSON
                rospy.loginfo("[GET] Data diterima: %s", str(data))
                
                # Contoh pengolahan data:
                coordinates = {
                    'tegangan': data.get('tegangan_asli'),
                    'baterai': data.get('persentase_baterai')
                }
                pub.publish(json.dumps(coordinates))
                
            except ValueError as e:
                rospy.logwarn("Gagal parse JSON: %s", str(e))
        else:
            rospy.logwarn("GET gagal, kode %d: %s", response.status_code, response.text)
            
    except requests.exceptions.RequestException as e:
        rospy.logwarn("Request error: %s", str(e))
    except Exception as e:
        rospy.logerr("Exception saat GET: %s", str(e))

if __name__ == "__main__":
    rospy.init_node("status_updater_node")
    pub = rospy.Publisher("/get_coordinates", String, queue_size=10)
    rospy.loginfo("Node aktif. Menunggu /post_status & publish ke /get_coordinates.")
    rate = rospy.Rate(0.2)  # Cek GET setiap 5 detik
    
    while not rospy.is_shutdown():
        update_sensor_data()
        get_coordinates()
        rate.sleep()
