#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Int32

def serial_listener():
    rospy.init_node('get_coord', anonymous=True)
    pub = rospy.Publisher('/command', Int32, queue_size=10)

    ser = serial.Serial('/dev/ttyUSB3', 9600, timeout=1)

    rospy.loginfo("Serial listening on /dev/ttyUSB2...")

    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            try:
                raw_data = ser.readline().decode('utf-8').strip()  # Bersihkan \n, spasi
                rospy.loginfo(f"Raw data: {raw_data}")

                # Pastikan hanya numerik
                if raw_data.isdigit() or (raw_data.startswith('-') and raw_data[1:].isdigit()):
                    msg = Int32(data=int(raw_data))
                    pub.publish(msg)
                else:
                    rospy.logwarn("Data bukan integer valid: %s", raw_data)

            except Exception as e:
                rospy.logerr(f"Error parsing data: {e}")

if __name__ == '__main__':
    try:
        serial_listener()
    except rospy.ROSInterruptException:
        pass
