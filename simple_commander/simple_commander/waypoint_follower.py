#! /usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from example_interfaces.srv import SetBool
from rclpy.node import Node

import rclpy

"""
Basic navigation demo to go to poses.
"""

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(SetBool, 'elevator_to_robot')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetBool.Request()

    def send_request(self, start_elevator):
        self.req.data = start_elevator # boolean for starting the elevator
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()

    navigator = BasicNavigator()

    route = [
        [1.10, 0.70, 0.707, 0.707],     #  90
        #[0.90, 0.70, 0.707, 0.707],    #  90
        [1.10, -0.70, 0.707, -0.707],   # -90
        [-1.10, -0.70, 0.707, -0.707],  # -90
        [-1.10, 0.70, 0.707, 0.707],    #  90
        [1.10, 0.7, 0.707, 0.707]       #  90
    ]

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    initial_pose.pose.orientation.z = 0.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()
    i = 1

    for pose in route:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.y = pose[0]
        goal_pose.pose.position.x = pose[1]
        goal_pose.pose.orientation.w = pose[2]
        goal_pose.pose.orientation.z = pose[3]
        navigator.goToPose(goal_pose)

        timer = 0
        while not navigator.isTaskComplete():
            timer += 1
            feedback = navigator.getFeedback()
            if feedback and timer % 10 == 0:
                print("Executing current waypoint: " +
                      str(i) + '/' + str(len(route)))
                
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
            minimal_client = MinimalClientAsync()
            response = minimal_client.send_request(True) # send request to start elevator
            minimal_client.get_logger().info(
                'Result of receive_bool: for %s' %(response.success)) # receive response from elevator
            minimal_client.destroy_node()
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')

        i += 1

    exit(0)


if __name__ == '__main__':
    main()