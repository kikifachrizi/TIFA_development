#!/usr/bin/env python
#kode ini tidak untuk di run

import rospy
import csv
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Twist
import threading

class CSVCommandExecutor:
    def __init__(self):
        rospy.init_node('csv_command_executor', anonymous=True)

        csv_path = rospy.get_param("~csv_path", "path/to/your/csv.csv")
        self.commands = self.load_csv(csv_path)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/command', Int32, self.command_callback)
        rospy.Subscriber('/emergency_stop', Bool, self.stop_callback)

        self.current_index = 0
        self.target_index = 0

        self.movement_thread = None
        self.new_command_event = threading.Event()
        self.stop_event = threading.Event()
        self.lock = threading.Lock()

        rospy.loginfo("CSV Command Executor READY.")
        rospy.spin()

    def load_csv(self, path):
        data = []
        with open(path, 'r') as file:
            reader = csv.reader(file)
            next(reader)  # skip header
            for row in reader:
                if len(row) >= 3:
                    try:
                        data.append([float(row[0]), float(row[1]), float(row[2])])
                    except ValueError:
                        rospy.logwarn("Invalid row skipped: %s", row)
        rospy.loginfo("CSV loaded with %d commands.", len(data))
        return data

    def command_callback(self, msg):
        with self.lock:
            self.target_index = msg.data
            if not (0 <= self.target_index < len(self.commands)):
                rospy.logwarn("Invalid target index %d. Ignored.", self.target_index)
                return

            rospy.loginfo("Received new command: index %d", self.target_index)

            if self.movement_thread and self.movement_thread.is_alive():
                self.new_command_event.set()
                rospy.loginfo("Interrupting current movement to apply new command...")
            else:
                self.new_command_event.clear()
                self.stop_event.clear()
                self.movement_thread = threading.Thread(target=self.move_to_index)
                self.movement_thread.start()

    def stop_callback(self, msg):
        if msg.data:
            rospy.logwarn("Emergency STOP received!")
            self.stop_event.set()
            self.new_command_event.set()  # interrupt also if moving
            self.publish_stop()

    def move_to_index(self):
        while not rospy.is_shutdown():
            with self.lock:
                start = self.current_index
                end = self.target_index
                step = 1 if end > start else -1
                indices = range(start, end + step, step)

            for idx in indices:
                if self.stop_event.is_set():
                    rospy.logwarn("Movement ABORTED due to emergency stop.")
                    return

                if self.new_command_event.is_set():
                    rospy.loginfo("New command detected. Restarting movement...")
                    self.new_command_event.clear()
                    break  # keluar dari loop dan lanjut dari target baru

                cmd = Twist()
                cmd.linear.x = self.commands[idx][0]
                cmd.linear.y = self.commands[idx][1]
                cmd.angular.z = self.commands[idx][2]
                self.cmd_pub.publish(cmd)

                rospy.loginfo("Moving to index %d: %s", idx, self.commands[idx])
                rospy.sleep(0.5)
                self.current_index = idx

            else:
                # Movement selesai tanpa interupsi
                rospy.loginfo("Arrived at index %d. Movement done.", self.current_index)
                self.publish_stop()
                return  # keluar dari thread

    def publish_stop(self):
        stop = Twist()
        self.cmd_pub.publish(stop)
