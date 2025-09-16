#!/usr/bin/env python3
import curses
import rospy
from std_msgs.msg import Float32MultiArray
import locale

# Pastikan format desimal (gunakan titik "." sebagai pemisah desimal)
locale.setlocale(locale.LC_NUMERIC, 'C')

def gui(stdscr):
    rospy.init_node("ncurses_pid_node", anonymous=True)
    pub = rospy.Publisher("/pid_gains", Float32MultiArray, queue_size=10)

    curses.curs_set(1)  # Tampilkan kursor
    stdscr.clear()

    while not rospy.is_shutdown():
        stdscr.addstr(2, 5, "input Kp: ")
        stdscr.refresh()
        curses.echo()
        kp = stdscr.getstr(2, 25, 10).decode('utf-8').strip()

        stdscr.addstr(3, 5, "input Ki: ")
        stdscr.refresh()
        ki = stdscr.getstr(3, 25, 10).decode('utf-8').strip()

        stdscr.addstr(4, 5, "input Kd: ")
        stdscr.refresh()
        kd = stdscr.getstr(4, 25, 10).decode('utf-8').strip()

        try:
            kp_val, ki_val, kd_val = float(kp), float(ki), float(kd)
            values = [kp_val, ki_val, kd_val]
            msg = Float32MultiArray()
            msg.data = values  # Format data harus benar untuk ROS

            rospy.loginfo(f"PID: Kp={kp_val}, Ki={ki_val}, Kd={kd_val}")
            pub.publish(msg)

            # stdscr.addstr(6, 5, f"Data Kp={kp_val}, Ki={ki_val}, Kd={kd_val} dikirim ke ROS!")
            stdscr.refresh()
        except ValueError:
            stdscr.addstr(6, 5, "Input tidak valid! Harap masukkan angka dengan format desimal.")
            stdscr.refresh()
        
        stdscr.getch()  # Tunggu input sebelum lanjut

if __name__ == "__main__":
    curses.wrapper(gui)
