# #!/usr/bin/env python3

# import rospy
# import serial
# from geometry_msgs.msg import Twist
# from std_msgs.msg import String

# ser = serial.Serial('/dev/ttyUSB1', 57600, timeout=1) 

# def cmd_vel_callback(msg):

#     rospy.loginfo("Velocities: x = %.2f, y = %.2f, z = %.2f", 
#                   msg.linear.x, msg.linear.y, msg.angular.z)
    
#     message = f"{round(msg.linear.x, 2)};{round(msg.linear.y,2)};{round(msg.angular.z,2)};\n"

#     ser.write(message.encode('utf-8'))

# def main():
#     rospy.init_node('manual_control', anonymous=True)
#     rospy.Subscriber('/turtle1/cmd_vel', Twist, cmd_vel_callback)

#     rospy.spin()

# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass


#!/usr/bin/env python3

import rospy
import serial
import threading
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

ser = serial.Serial('/dev/ttyUSB1', 57600, timeout=1)  #ftdi
# ser2 = serial.Serial('/dev/ttyUSB0', 19200, timeout=1)  #cp210x

kp_ = 0.0
ki_ = 0.0
kd_ = 0.0
data =""
pub = None

def get_params(msg):
    global kp_, ki_, kd_
    kp_ = msg.data[0]
    ki_ = msg.data[1]
    kd_ = msg.data[2]

def cmd_vel_callback(msg):
    global kp_, ki_, kd_

    rospy.loginfo(
    "Sending Data: x=%.2f, y=%.2f, z=%.2f, kp=%.2f, ki=%.2f, kd=%.2f",
    round(msg.linear.x, 2) * 2.25,
    round(msg.linear.y, 2) * 2.25,
    round(msg.angular.z, 2) * 25,
    round(kp_, 2),
    round(ki_, 2),
    round(kd_, 2)
    )

    
    message = f"{round(msg.linear.x, 2) * 2.25} {round(msg.linear.y, 2) * 2.25} {round(msg.angular.z, 2) * 25} {round(kp_, 2)} {round(ki_, 2)} {round(kd_, 2)};\n"
    ser.write(message.encode('utf-8'))

# def read_serial_data():
#     global data
#     while not rospy.is_shutdown():
#         if ser2.in_waiting > 0:
#             data = ser2.readline().decode('utf-8') 
#             rospy.loginfo(f"Received from /dev/ttyUSB2: {data}")
#             pub.publish(data)



def main():
    global pub
    rospy.init_node('manual_control', anonymous=True)
    rospy.Subscriber('/turtle1/cmd_vel', Twist, cmd_vel_callback)
    rospy.Subscriber('/pid_gains', Float32MultiArray, get_params)
    # pub = rospy.Publisher("/raw_data_pid", String, queue_size=10)

    # thread = threading.Thread(target=read_serial_data, daemon=True)

    # thread.start()


    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
