#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

from std_msgs.msg import String


class CommandVelSub(Node):

    def __init__(self):
        super().__init__('turtlebot_commander_nav')

        self.navigator = BasicNavigator()
        self.complete = String()
        self.subscription = self.create_subscription(
            PoseStamped, 'cmd_waffle', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(String, 'waffle_response', 10)

    def listener_callback(self, msg):

        full_route = [
            [msg.pose.position.x, msg.pose.position.y, msg.pose.orientation.z]]

        # # Set our demo's initial pose
        # initial_pose = PoseStamped()
        # initial_pose.header.frame_id = 'map'
        # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        # initial_pose.pose.position.x = 0.30
        # initial_pose.pose.position.y = 0.00
        # initial_pose.pose.orientation.z = 0.0
        # initial_pose.pose.orientation.w = 0.1
        # navigator.setInitialPose(initial_pose)

        # Wait for navigation to fully activate
        self.navigator.waitUntilNav2Active()

        # # Do security route until dead
        # while rclpy.ok():
        # Send our route
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.orientation.w = 1.0

        for pt in full_route:
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.orientation.z = pt[2]
            self.navigator.goToPose(pose)

            while not self.navigator.isTaskComplete():
                pass

        # Later to check back x:0.0 and y:0.0
        if ((pose.pose.position.x == 2.0) & (pose.pose.position.y == 0.0)):
            self.complete.data = "Patrol Complete"
            self.publisher_.publish(self.complete)

        # if((pose.pose.position.x == 0.632) & (pose.pose.position.y == 2.022)):
        #     self.complete.data = "Pose Reached"
        #     self.publisher_.publish(self.complete)

        elif (((pose.pose.position.x == 1.18) & (pose.pose.position.y == -1.01)) or 
              ((pose.pose.position.x == 1.523) & (pose.pose.position.y == 1.212)) or 
              ((pose.pose.position.x == 1.79) & (pose.pose.position.y == 1.88)) or 
              ((pose.pose.position.x == 1.79) & (pose.pose.position.y == 1.02)) or 
              ((pose.pose.position.x == 0.63) & (pose.pose.position.y == 1.88)) or 
              ((pose.pose.position.x == 0.63) & (pose.pose.position.y == 1.02))):

            self.complete.data = "Pose Reached"
            self.publisher_.publish(self.complete)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = CommandVelSub()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
