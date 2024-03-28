#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def image_callback(msg):
    try:

        bridge = CvBridge()

        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        cv2.imwrite('saved_image.png', cv_image)
        rospy.loginfo("Image saved successfully")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

def main():
    rospy.init_node('image_listener', anonymous=True)
    rospy.Subscriber("/yolov5/image_out", Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
