from geometry_msgs.msg import PoseStamped
from copy import deepcopy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from visualization_msgs.msg import Marker
from std_msgs.msg import String


class RecMisplacedPose(Node):

    def __init__(self):
        super().__init__('patrol')
        self.navigator = BasicNavigator()
        self.pose= PoseStamped()
        self.store = PoseStamped()
        # self.subscription = self.create_subscription(Marker,'tag_pose_marker',self.listener_callback,10)
        # self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(PoseStamped, 'cmd_waffle', 10)
        self.end = False

        full_route = [[2.0, 0.00, 0.505]] #,[2.17, 2.83, 2.36], [0.00, 3.00, -1.57], [0.00, 0.00, 0.00]]
      
        # Wait for navigation to fully activate
        self.navigator.waitUntilNav2Active()

        for pt in full_route:
            self.pose.pose.position.x = pt[0]
            self.pose.pose.position.y = pt[1]
            self.pose.pose.orientation.z = pt[2]
            self.publisher_.publish(self.pose)
            while not self.navigator.isTaskComplete():
                pass
        

        # self.end = True


    # def listener_callback(self, tag_marker):

    #     self.store.pose.position.x = float(tag_marker.pose.position.x)
    #     self.store.pose.position.y = float(tag_marker.pose.position.y)
    #     self.store.pose.orientation.z = float(tag_marker.pose.orientation.z)

    #     if(self.end == True):
    #         print("Patrol ended")

    #         if(self.navigator.isTaskComplete()):
    #             #Send the aruco misplacechair to listener node
    #             self.publisher_.publish(self.store)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = RecMisplacedPose()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
