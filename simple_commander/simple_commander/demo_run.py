#! /usr/bin/env python3
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from example_interfaces.srv import SetBool
from rclpy.node import Node
import time
import math

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

class PoseTracker(Node):
    def __init__(self):
        super().__init__('pose_tracker')
        self.current_pose = Pose()
        self.navigator = BasicNavigator()
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.amcl_pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/amcl_pose', 
            self.pose_callback, 
            10
        )

        self.demo_route = [
            [1.10, 0.70, 0.707, 0.707],     #  90 top-right
            [1.20, -0.60, 0.0, 1.0],        # 180 top-left
            [-1.20, 0.60, 1.0, 0.0],        #   0 bottom-right
            [1.10, 0.70, 0.707, 0.707],     #  90 top-right
        ]

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def move_backwards(self, linear_velocity, distance):
        twist = Twist()
        twist.linear.x = -linear_velocity
    
        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y

        print(f"Moving backwards for {distance} meters")

        while True:
            self.cmd_vel_publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)

            current_pose = self.current_pose
            distance_moved = math.sqrt((current_pose.position.x - start_x) ** 2 
                                    + (current_pose.position.y - start_y) ** 2)
            if distance_moved >= distance:
                break

        twist.linear.x = 0.0
        self.cmd_vel_publisher.publish(twist)

        print("Finished moving backwards")

    def navigate(self):
        self.navigator.waitUntilNav2Active()
        i = 1

        for pose in self.demo_route:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.y = pose[0]
            goal_pose.pose.position.x = pose[1]
            goal_pose.pose.orientation.w = pose[2]
            goal_pose.pose.orientation.z = pose[3]

            self.goal_publisher.publish(goal_pose)

            self.navigator.goToPose(goal_pose)

            timer = 0
            while not self.navigator.isTaskComplete():
                timer += 1
                feedback = self.navigator.getFeedback()
                if feedback and timer % 10 == 0:
                    print("Executing current waypoint: " +
                          str(i) + '/' + str(len(self.demo_route)))
                rclpy.spin_once(self, timeout_sec=0.1)
                    
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
                minimal_client = MinimalClientAsync()
                response = minimal_client.send_request(True) # send request to start elevator
                minimal_client.get_logger().info(
                    'Result from elevator: %s' %(response.success)) # receive response from elevator
                minimal_client.destroy_node()
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')

            rclpy.spin_once(self, timeout_sec=1.0)
            self.move_backwards(linear_velocity=0.10, distance=0.6)

            i += 1

def main():
    rclpy.init()
    pose_tracker = PoseTracker()
    pose_tracker.navigate()
    rclpy.spin(pose_tracker)
    pose_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()