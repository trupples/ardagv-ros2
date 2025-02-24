#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

"""
Basic navigation demo to go to poses.
"""


def main():
    rclpy.init()

    navigator = BasicNavigator()

    route = [
        [1.10, 0.70, -0.707, -0.707],
        [1.10, -0.70, -0.707, -0.707],
        [-1.10, -0.70, -0.707, -0.707],
        [-1.10, 0.70, -0.707, -0.707],
        [1.10, 0.7, -0.707, -0.707]
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
            #Duration(seconds=0.5).sleep()

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        
        #sleep(Duration(seconds=1.0))
        i += 1

    exit(0)


if __name__ == '__main__':
    main()