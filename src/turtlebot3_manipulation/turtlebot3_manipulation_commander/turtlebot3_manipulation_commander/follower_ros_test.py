#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import numpy as np
import cv2
import time

class SimplePubSub(Node):
    def __init__(self):
        super().__init__('simple_pub_sub')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist = Twist()
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()

        self.subscription = self.create_subscription(
            Image, 'pi_camera/image_raw', self.img_callback, 10)
        self.subscription
        self.br = CvBridge()

        self.moveforward = False

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret == True:
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))

    def img_callback(self, data):

        img = self.br.imgmsg_to_cv2(data)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # blur = cv2.GaussianBlur(hsv,(10,10),cv2.BORDER_DEFAULT) #cv2.fastNlMeansDenoisingColored(hsv, None, 40,20,7,21)
        # h, w, d = mask.shape #h=480,w=640

        # convert to hsv colorspace
        # lower bound and upper bound for blue color
        lower_bound = np.array([100, 00, 0])
        upper_bound = np.array([130, 255, 255])

        # find the colors within the boundaries
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        areaDetect = np.zeros((480, 640), dtype="uint8")

        offset_width = 10
        cv2.rectangle(areaDetect, (0+offset_width, 300), (250, 350), (255), -1)
        cv2.rectangle(areaDetect, (380, 300), (640-offset_width, 350), (255), -1)
        # cv2.rectangle(areaDetect, (0, 300), (640, 400), (255), -1)

        # Find Canny edges
        edged = cv2.Canny(areaDetect, 30, 200)

        #To see the area of interest zone
        contours, hierarchy = cv2.findContours(
            edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(img, contours, -1, (0, 0, 255), 2)

 
        # calculate moments of masked image
        filter = cv2.bitwise_and(areaDetect, mask)

        filter = np.vstack(filter)


        M = cv2.moments(filter)
        cv2.imshow("filter",filter)


        if M['m00'] > 0:

            if self.moveforward == False:
                fwd_speed = 0.0
                turn_speed = 1000

            elif self.moveforward == True:
                fwd_speed = 0.15
                turn_speed = 5000
                cv2.line(img, (320,280),(320,400),(255,0,0),2)
                cv2.line(img, (cx-40,280),(cx-40,400),(0,255,0),2)
                cv2.line(img, (cx+40,280),(cx+40,400),(0,255,0),2)

            else:
                fwd_speed = 0.0
                turn_speed = 0.0


            # calculate x,y coordinate to find centroid of image
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            # to highlight the center
            cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)

            # CONTROL starts
            err = int(cx - 640/2)  # To centralise in middle of screen
            self.twist.linear.x = fwd_speed
            # Tune the divisor to adjust speed of turning
            self.twist.angular.z = -float(err) / turn_speed
            self.cmd_vel_pub.publish(self.twist)
            # CONTROL ends

            if (err >= -3 & err <= 3):
                self.moveforward = True
                self.twist.linear.x = fwd_speed
                self.twist.angular.z = -float(err) / turn_speed
                self.cmd_vel_pub.publish(self.twist)

        elif M['m00'] <= 0:
            time.sleep(1)
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)

        cv2.imshow("camera", img)
        # cv2.imshow("mask", mask)

        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    simple_pub_sub = SimplePubSub()
    rclpy.spin(simple_pub_sub)
    simple_pub_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except rclpy.ROSInterruptException:
        pass
