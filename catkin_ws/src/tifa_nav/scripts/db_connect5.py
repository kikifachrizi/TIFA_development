#!/usr/bin/env python3
import rospy
import requests
import json
from std_msgs.msg import Int32, String
import time

# === Konfigurasi Server ===
SERVER_HOST = "http://172.20.10.10/"
UPDATE_PATH = "tes_app/receive.php"
GET_PATH = "tes_app/deliver.php"

# === Konfigurasi Data ===
id1 = 125
id2 = 126
id3 = 127  # id baris di tabel orders yang akan diupdate

# === Variabel Global ===
status_to_post = 0
coordinates = ""

def status_callback(msg):
    global id1, id2, id3
    status = msg.data
    #print(status)
    get_coordinates()
    update_status_to_server(id1, id2, id3, status)

def update_status_to_server(id1, id2, id3, status):
    #global id1, id2, id3
    try:
        payload1 = {"id": id1, "status": status}
        response1 = requests.post(SERVER_HOST + UPDATE_PATH, data=payload1)
        #print(response1.status)

        payload2 = {"id": id2, "status": status}
        response2 = requests.post(SERVER_HOST + UPDATE_PATH, data=payload2)

        payload3 = {"id": id3, "status": status}
        response3 = requests.post(SERVER_HOST + UPDATE_PATH, data=payload3)

        if response1.status_code == 200:
            rospy.loginfo("[UPDATE] Status %s terkirim ke id %d: %s", status, id1, response1.text)
        else:
            rospy.logwarn("[UPDATE] Gagal dengan kode %d: %s", response1.status_code, response1.text)
        if response2.status_code == 200:
            rospy.loginfo("[UPDATE] Status %s terkirim ke id %d: %s", status, id2, response2.text)
        else:
            rospy.logwarn("[UPDATE] Gagal dengan kode %d: %s", response2.status_code, response2.text)
        if response3.status_code == 200:
            rospy.loginfo("[UPDATE] Status %s terkirim ke id %d: %s", status, id3, response3.text)
        else:
            rospy.logwarn("[UPDATE] Gagal dengan kode %d: %s", response3.status_code, response3.text)
    except Exception as e:
        rospy.logerr("Gagal UPDATE status: %s", str(e))

def get_coordinates():
    global id1, id2, id3
    print("Mengambil koordinat dari server...")
    try:
        response = requests.get(SERVER_HOST + GET_PATH)
        if response.status_code == 200:
            data = json.loads(response.text)
            print("len data : ", len(data))
            if isinstance(data, list) and len(data) > 0:
                #print("B")
                coordinates = data[0].get("coordinates", "")
                id1 = int(data[0].get("id", id1))
                id2 = int(data[1].get("id", id2))
                id3 = int(data[2].get("id", id3))
                print("idx 0 : ", id1, "idx 1 : ", id2, "idx 2 : ", id3)
                pub.publish(coordinates)
                # rospy.loginfo("[GET] Coordinates diterima: %s", coordinates)
            else:
                rospy.logwarn("Format response GET tidak sesuai.")
        else:
            rospy.logwarn("GET gagal, kode %d", response.status_code)
    except Exception as e:
        rospy.logwarn("Exception saat GET: %s", str(e))

if __name__ == "__main__":
    rospy.init_node("status_updater_node")
    pub = rospy.Publisher("/get_coordinates", String, queue_size=10)
    rospy.Subscriber("/post_status", Int32, status_callback)
    get_coordinates()

    rospy.loginfo("Node aktif. Menunggu /post_status & publish ke /get_coordinates.")
    #get_coordinates()
    rate = rospy.Rate(0.2)  # Cek GET setiap 5 detik

    rospy.spin()
