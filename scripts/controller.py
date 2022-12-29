#! /usr/bin/python3

import argparse
import curses
import sys

import rospy
from geometry_msgs.msg import Twist


def main(stdscr: curses.window):
    rospy.init_node("controller")

    publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    stdscr.clear()
    curses.halfdelay(1)

    last_key = None
    while not rospy.is_shutdown():
        c = stdscr.getch()
        if c != last_key:
            last_key = c
            continue
        last_key = c

        vel_msg = Twist()
        if c == curses.KEY_UP:
            vel_msg.linear.x = 1.0
        elif c == curses.KEY_DOWN:
            vel_msg.linear.x = -1.0
        elif c == curses.KEY_RIGHT:
            vel_msg.angular.z = -1.0
        elif c == curses.KEY_LEFT:
            vel_msg.angular.z = 1.0
        elif c == ord("q"):
            return

        stdscr.addstr(0,0, f"Linear: {vel_msg.linear.x}, Angular: {vel_msg.angular.z}")
        stdscr.refresh()

        publisher.publish(vel_msg)

if __name__ == "__main__":
    curses.wrapper(main)
