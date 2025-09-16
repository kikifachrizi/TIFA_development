#!/usr/bin/env python3
import firebase_admin
from firebase_admin import credentials, db
from datetime import datetime
import time
import serial

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  

# Initialize Firebase
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

# Function to update Firebase with fixed values
def send_fixed_values():
    try:
        line = ser.readline().decode().strip()
        values = line.split(';')

        ref = db.reference("/status")
        
        ref.update({
            "makanan": "tung tung sahur",
            "pintu": "asasino cappucino",
            "tray": "brbr patabim",
            "last_updated": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        })
        
        print("✅ Data sent to Firebase at", datetime.now().strftime("%H:%M:%S"))
        return True
    except Exception as e:
        print(f"❌ Error updating Firebase: {e}")
        return False

if __name__ == "__main__":
    if not init_firebase():
        exit(1)
    
    print("Starting continuous Firebase updates...")
    print("Press Ctrl+C to stop")
    
    try:
        while True:
            if send_fixed_values():
                time.sleep(1)  # Wait 1 second before next update
            else:
                time.sleep(5)  # Wait longer if error occurred
    except KeyboardInterrupt:
        print("\nStopped by user")