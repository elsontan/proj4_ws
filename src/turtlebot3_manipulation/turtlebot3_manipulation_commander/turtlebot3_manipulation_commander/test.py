import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('test')
        self.publisher_ = self.create_publisher(PoseStamped, 'cmd_waffle_pi', 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

    # def timer_callback(self):
        msg = PoseStamped()

        msg.pose.position.x = 1.63
        msg.pose.position.y = 0.00
        msg.pose.orientation.z = -0.785
        
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()