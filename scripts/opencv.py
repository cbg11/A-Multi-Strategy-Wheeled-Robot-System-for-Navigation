#!/usr/bin/env python3

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def talker(data):
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    cv_image = cv2.resize(cv_image, (400, 400)) 
    cv2.imshow("Raw Image", cv_image)
    cv2.waitKey(3)

def main():
    rospy.init_node("my_planner_node")
    img_sub = rospy.Subscriber("/camera/image_raw", Image, talker)
    rospy.spin()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
