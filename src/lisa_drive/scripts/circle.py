#!/usr/bin/env python
import roslib; roslib.load_manifest('lisa_drive')
import rospy
import cv2
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

	def __init__(self):
		self.image_pub = rospy.Publisher('/img_pub', Image, queue_size=10)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber('/dvs/image_raw', Image, self.callback)

	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		(rows,cols,channels) = cv_image.shape
		if cols > 60 and rows > 60:
			cv2.circle(cv_image, (50,50), 10, 255)

		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
			print(e)


def main(args):
	rospy.init_node('image_raw_talker', anonymous=True)
	image_converter()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()
	
    	

if __name__ == '__main__':
	main(sys.argv)
