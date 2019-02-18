#!/usr/bin/env python
import roslib; roslib.load_manifest('lisa_drive')
import rospy
import argparse
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import numpy as np


class Plotter:

    def __init__(self, num_msgs):
        self.num_msgs = num_msgs

        self.sub_cmd_vel = rospy.Subscriber('/cmd_vel_2', Twist, self.callback_cmd_vel)
        self.sub_odom = rospy.Subscriber('/DVS346/odom', Odometry, self.callback_odom)

        self.curr_cmd_msg = 0
        self.prev_cmd_msg = 0
        self.curr_odom_msg = 0

        self.count = 0
        self.msgs = []

        self.cmd_msgs = []
        self.odom_msgs = []

        self.run()

    def callback_cmd_vel(self, data):
        # Store data
        self.curr_cmd_msg = data.linear.x

    def callback_odom(self, data):

        lin_x = data.twist.twist.linear.x
        lin_y = data.twist.twist.linear.y
        lin_z = data.twist.twist.linear.z

        vel_comps = np.array([lin_x, lin_y, lin_z]).reshape((3, 1))
        vel_comps = vel_comps.reshape((3, 1))
        spd = np.linalg.norm(vel_comps)

        # Store data
        self.curr_odom_msg = spd

    def run(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.cmd_msgs.append(self.curr_cmd_msg)
            self.odom_msgs.append(self.curr_odom_msg)

            self.prev_cmd_msg = self.curr_cmd_msg

            self.count += 1
            self.msgs.append(self.count)

            if len(self.cmd_msgs) >= self.num_msgs:
                self.cmd_msgs = self.cmd_msgs[-self.num_msgs:]
            if len(self.odom_msgs) >= self.num_msgs:
                self.odom_msgs = self.odom_msgs[-self.num_msgs:]
            if len(self.msgs) >= self.num_msgs:
                self.msgs = self.msgs[-self.num_msgs:]

            # Plot
            plt.figure(1)
            plt.clf()
            plt.subplot(211)
            plt.plot(self.msgs, self.cmd_msgs, '-o', color='b')
            #plt.ylim(np.mean(self.cmd_msgs) - .05, np.mean(self.cmd_msgs) + .05)
            plt.title("Power subscribed from /cmd_vel in linear.x direction")
            plt.subplot(212)
            plt.plot(self.msgs, self.odom_msgs, '-o', color='r')
            plt.ylim(np.mean(self.odom_msgs) - .5, np.mean(self.odom_msgs) + .5)
            plt.title('Speed message received from /DVS346/odom in linear.x direction')
            plt.draw()
            plt.tight_layout()
            plt.pause(0.0000000001)

            r.sleep()


def main():
    parser = argparse.ArgumentParser(description='code')
    parser.add_argument('-m', '--number_messages', type=int, default=50,
                        help='Number of messages to use at a time')
    args = parser.parse_args()
    rospy.init_node('plotter_util', anonymous=True)
    Plotter(args.number_messages)


if __name__ == '__main__':
    main()
