from geometry_msgs.msg import PoseStamped
from copy import deepcopy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def main():
    rclpy.init()

    navigator = BasicNavigator()


    full_route = [
        # [-1.656, 1.908, 1.57, 0.0], 
        # [-3.746, 1.18,-1.57, 0.0], 
        [-1.40, 0.62,-0.00, 0.00, 0.00, 1.57], 
]

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
    navigator.waitUntilNav2Active()

    
    # # Do security route until dead
    # while rclpy.ok():
        # Send our route
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.orientation.w = 1.0
    for pt in full_route:
        pose.pose.position.x = pt[0]
        pose.pose.position.y = pt[1]
        pose.pose.position.z = pt[2]
        pose.pose.orientation.x = pt[0]
        pose.pose.orientation.y = pt[1]
        pose.pose.orientation.z = pt[2]
        navigator.goToPose(pose)
    

            # while not navigator.isTaskComplete():
            #     pass


if __name__ == '__main__':
    main()

