#!/usr/bin/env python3

import rospy

import cv2
import imutils
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from std_msgs.msg import Bool


class NumberCounter:
    def __init__(self):
        self.counter = 0
        self.pub_img = rospy.Publisher("camera1/image_processed", Image, queue_size=1)
        self.pub_vis = rospy.Publisher("camera1/ball_visible", Bool, queue_size=1)
        self.number_subscriber = rospy.Subscriber("camera1/image_raw", Image, self.callback_raw_image)

        self.bridge = CvBridge()
        
    def callback_raw_image(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        greenLower = (50, 50, 20)
        greenUpper = (70, 255, 255)

        blurred = cv2.GaussianBlur(self.image, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        visible = False

                # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                visible = True
                cv2.circle(self.image, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(self.image, center, 5, (0, 0, 255), -1)

        self.pub_img.publish((self.bridge.cv2_to_imgmsg(self.image, 'bgr8')))
        self.pub_vis.publish(visible)


if __name__ == '__main__':
    rospy.init_node('number_counter')
    NumberCounter()
    rospy.spin()


