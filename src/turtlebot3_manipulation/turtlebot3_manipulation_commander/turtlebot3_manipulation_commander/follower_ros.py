#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
import cv2
import time

class SimplePubSub(Node):
    def __init__(self):
        super().__init__('simple_pub_sub')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist = Twist()

        # self.poseSub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # self.poseSub # prevent unused variable warning

        self.pose = Odometry()

        # self.timer = self.create_timer(0.1, self.timer_callback)

        # self.cap = cv2.VideoCapture(0)
        # self.br = CvBridge()

        self.subscription = self.create_subscription(Image, '/usb_cam_1/image_raw', self.img_callback, 10) #/pi_camera/image_raw
        self.subscription # prevent unused variable warning

        self.br = CvBridge()

        self.publisher_ = self.create_publisher(String, 'waffle_response', 10)

        self.moveforward = False
        self.complete = String()
        self.end = False

    # def odom_callback(self,msg):
    #     # To track the position of the robot
    #     x = ('%s' % ('%.3g' % float(msg.pose.pose.position.x)))
    #     y = ('%s' % ('%.3g' % float(msg.pose.pose.position.y)))
    #     z = ('%s' % ('%.3g' % float(msg.pose.pose.position.z)))

    #     arr = np.array(["x: "+x,"y: "+y,"z: "+z])
    #     np.set_printoptions(suppress = True)

        # print(arr)
        
        # orientation = self.pose.pose.orientation
        # (posx, posy, posz) = (pos.x, pos.y, pos.z)
        # (qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z, orientation.w)
        

    # def timer_callback(self):
    #     ret, frame = self.cap.read()
    #     if ret == True:
    #         self.publisher_.publish(self.br.cv2_to_imgmsg(frame))

    def img_callback(self, data):
        
        img = self.br.imgmsg_to_cv2(data)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # h, w, d = mask.shape #h=480,w=640

        # convert to hsv colorspace
        # lower bound and upper bound for blue color
        lower_bound = np.array([100, 100, 100])
        upper_bound = np.array([140, 200, 200])

        # find the colors within the boundaries
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        areaDetect = np.zeros((480, 640), dtype="uint8")

        offset_width =50
        cv2.rectangle(areaDetect, (0, 260), (250+offset_width, 350), (255), -1)
        cv2.rectangle(areaDetect, (380-offset_width, 260), (640, 350), (255), -1)
        # cv2.rectangle(areaDetect, (0, 300), (640, 400), (255), -1)

        # Find Canny edges
        edged = cv2.Canny(areaDetect, 30, 200)

        #To see the area of interest zone
        contours, hierarchy = cv2.findContours(
            edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(img, contours, -1, (0, 0, 255), 2)

 
        # calculate moments of masked image
        filter = cv2.bitwise_and(areaDetect, mask)


        M = cv2.moments(filter)
        cv2.imshow("filter",filter)

        # print(self.pose.position, self.pose.orientation)

        if (M['m00'] > 0)&(self.end == False):

            #print(self.moveforward)

            if self.moveforward == False:
                fwd_speed = 0.0
                turn_speed = 1800

            elif self.moveforward == True:
                fwd_speed = 0.15
                turn_speed = 20000
                self.twist.angular.z = 0.0
                cv2.line(img, (320,280),(320,400),(255,0,0),2)


            # calculate x,y coordinate to find centroid of image
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            # to highlight the center
            cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)
            # cv2.line(img, (cx-40,280),(cx-40,400),(0,255,0),2)
            # cv2.line(img, (cx+40,280),(cx+40,400),(0,255,0),2)
            # CONTROL starts
            err = 0
            err = int(cx - 640/2)  # To centralise in middle of screen
            self.twist.linear.x = fwd_speed
            # Tune the divisor to adjust speed of turning
            self.twist.angular.z = -float(err) / turn_speed
            self.cmd_vel_pub.publish(self.twist)
            # CONTROL ends

            if ((err > -5) & (err < 5)):
                self.moveforward = True
                self.twist.linear.x = fwd_speed
                self.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)

        elif ((M['m00'] <= 0) & (self.twist.linear.x != 0)):
            self.moveforward = False
            time.sleep(4)
            self.end = True
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            self.complete.data = 'Pose Reached'
            self.publisher_.publish(self.complete)

        if(self.end):
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
