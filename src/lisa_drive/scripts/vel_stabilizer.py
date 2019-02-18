#!/usr/bin/env python
import roslib; roslib.load_manifest('lisa_drive')
import rospy
import sys
import argparse
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import matplotlib.pyplot as plt


class VelocityControl:

	def __init__(self, target_speed, alpha):

		self.target_speed = target_speed
		self.alpha = alpha

		self.pub = rospy.Publisher('/cmd_vel_2', Twist, queue_size=20)
		self.sub = rospy.Subscriber('/DVS346/odom', Odometry, self.callback)

		plt.ion()
		plt.show()
		# self.sub = rospy.Subscriber("/cmd_vel", Twist, self.callback)
		self.speed = 0.05
		self.messages = 0

		self.power = 0

	def publish(self,lin_pow):
		msg = Twist()
		msg.linear.x = lin_pow

		# Publish the message
		self.pub.publish(msg)

		# Log info
		# rospy.loginfo("Message published to /cmd_vel_2: \n\tLinear.x: %f\n\tAngular.z: %f", lin_pow, ang_pow)

	def callback(self, data):

		lin_x = data.twist.twist.linear.x
		lin_y = data.twist.twist.linear.y
		lin_z = data.twist.twist.linear.z

		# Log info
		# rospy.loginfo("Message received from /DVS346/odom: \n\tLinear components: [%f, %f, %f]" % (lin_x, lin_y, lin_z))

		vel_comps = np.array([lin_x, lin_y, lin_z]).reshape((3, 1))
		vel_comps = vel_comps.reshape((3, 1))
		spd = np.linalg.norm(vel_comps)

		self.messages = self.messages + 1

		self.power += ((self.target_speed - spd) * self.alpha)

		# Invoke method to publish message
		self.publish(self.power)


def main():
	parser = argparse.ArgumentParser(description='code')
	parser.add_argument('-t', '--target_speed', type=float, default=0.3, help='Target speed of vehicle')
	parser.add_argument('-a', '--alpha', type=float, default=0.1, help='Alpha value for updating power')
	args = parser.parse_args()
	rospy.init_node('plotter_util', anonymous=True)
	VelocityControl(args.target_speed, args.alpha)
	rospy.spin()


if __name__ == '__main__':
	main()
