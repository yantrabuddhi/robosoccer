#!/usr/bin/env python
__author__ = 'ss'

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

class BallTracker:
    NODE_NAME = 'ball_tracker'

    def __init__(self):
        # self.hsv_lower = np.array([12, 91, 84], np.uint8)
        # self.hsv_upper = np.array([96, 204, 255], np.uint8)
        self.hsv_lower = np.array([12, 188, 122], np.uint8)
        self.hsv_upper = np.array([61, 213, 255], np.uint8)
        self.publisher = rospy.Publisher('/ball_pose', Point, queue_size=10)
        self.bridge = CvBridge()
        self.image_subscriber = rospy.Subscriber("/cv_camera/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            ball_pose = self.detect_ball(frame)
            if ball_pose != None:
                self.publisher.publish(ball_pose)
            cv2.imshow("Live Feed", frame)
            cv2.waitKey(10)
        except CvBridgeError, e:
            print e

    def detect_ball(self, frame):
        buffer_frame = cv2.GaussianBlur(frame, (5, 5), 0)  # Gaussian Blur
        buffer_frame = cv2.cvtColor(buffer_frame, cv2.COLOR_BGR2HSV)  # Convert to HSV
        buffer_frame = cv2.inRange(buffer_frame, self.hsv_lower, self.hsv_upper)  # Check if each pixel is in boundary
        buffer_frame = cv2.dilate(buffer_frame, np.ones((20, 20), "uint8"))  # Dilate
        contours, hierarchy = cv2.findContours(buffer_frame, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        ball_contour = self.largest_contour(contours)

        if ball_contour != None:
            # Filter out smaller contours
            if cv2.contourArea(ball_contour) > 1500:
                cv2.drawContours(frame, [ball_contour], 0, np.array([200, 100, 100]), 2)
                height, width, channels = frame.shape
                return Point(float(ball_contour[0][0][0]) / width, float(ball_contour[0][0][1]) / height, 0)

    def largest_contour(self, contours):
        max_contour = [0, None]  # [Area, Index]
        if contours:
            for idx, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if area > max_contour[0]:
                    max_contour = [area, contour]
            return max_contour[1]


if __name__ == '__main__':
    rospy.loginfo("Starting " + BallTracker.NODE_NAME)
    BallTracker()
    rospy.init_node(BallTracker.NODE_NAME, anonymous=False)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Stopping " + BallTracker.NODE_NAME)
    cv2.destroyAllWindows()
