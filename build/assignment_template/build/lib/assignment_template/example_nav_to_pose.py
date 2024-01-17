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
import math
import time 
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler
from tf2_ros import Buffer, TransformListener
import tf2_ros

import tf_transformations

class Navigation(Node):
    def __init__(self):
        super().__init__('autonomous_navigator')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.sequence = True
        self.navigator = BasicNavigator()

    #path_planning based on following waypoints(designed specific ford this map)
    # get the position from frame
    def get_pos(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            # quat = trans.transform.rotation
            # euler = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            # theta = euler[2]  # Assuming Z-axis rotation (yaw)
            return [0, 0]
        except Exception as e:
            self.get_logger().error("Failed to get robot pose")
            return None

    def path_planning(self,robot_pose):
        # defining waypoints and sequence
        waypoints = []
        # manually defined to be slightly smaller than central to avoid collision 
        waypoints = [
                    [-0.2,0.0,0.0],
                    [1.2,0.0,0.0],
                    [1.2,-1.0,0.0],
                    [-0.2,-1.0,0.0],
                    [-0.2,0.0,0.0],
                    [-1.2,0.0,0.0],
                    [-1.2,-1.0,0.0],
                    [-0.2,-1.0,0.0],
                    [-0.2,0.0,0.0]
                    ]
        id = self.closest_waypoint(waypoints,robot_pose) 
        if id != 0: 
            waypoints.pop()
        rotated_lst = waypoints[id:] + waypoints[:id]
        if self.sequence == True:
            new_waypoints = rotated_lst
            new_waypoints.append(new_waypoints[0])
        else: 
            reversed_lst = [rotated_lst[0]] + rotated_lst[:0:-1]
            new_waypoints = reversed_lst
            new_waypoints.append(new_waypoints[0])
        print('WAYPOINTS', id, new_waypoints)
        return new_waypoints
    # find the closest possible waypoint to geft started
    def closest_waypoint(self,waypoints,robot_pose):
        # robot_pose = self.get_pos()
        if robot_pose == None:
            robot_pose =[0.0,0.0,]
        robot_position = [robot_pose[0],robot_pose[1]]
        robot_orientation = robot_pose[2]
        closest_waypoint = None
        
        min_distance = float('inf')
        waypoint_id  = 0
        for waypoint in waypoints:
            distance = self.calculate_distance(robot_position, waypoint)
            angle = self.calculate_angle(robot_position, waypoint)
            normalized_orientation = self.normalize_angle(robot_orientation)
            relative_angle = self.normalize_angle(angle-normalized_orientation)
            if distance < min_distance and np.abs(relative_angle)<= np.pi/2:
                min_distance = distance
                closest_waypoint = waypoint
                closest_id = waypoint_id
                closest_angle = normalized_orientation
            waypoint_id += 1

        new_angle = self.calculate_angle(waypoints[closest_id],waypoints[closest_id + 1])
        new_relative_angle = self.normalize_angle(new_angle-closest_angle)
        if np.abs(new_relative_angle)<=np.pi/2:
            self.sequence = True
        else: 
            self.sequence = False
        return closest_id
    def calculate_distance(self, point1, point2):
        return math.sqrt((point2[0
        ] - point1[0])**2 + (point2[1] - point1[1])**2)
    def calculate_angle(self, point1, point2):
        direction_vector = [point2[0] - point1[0], point2[1] - point1[1]]
       
        waypoint_angle = math.atan2(direction_vector[1], direction_vector[0])
        normalized_angle = math.atan2(math.sin(waypoint_angle), math.cos(waypoint_angle))
        return normalized_angle
    def normalize_angle(self,angle):
        normalized_angle = math.atan2(math.sin(angle), math.cos(angle))
        return normalized_angle
    def navigate(self,waypoints):
        waypoints_copy = waypoints.copy()
        for i in range(len(waypoints_copy)):
            if i == 0:
                direct_current = self.calculate_angle(waypoints[-1],waypoints[i])
                direct_next = self.calculate_angle(waypoints[i],waypoints[i+1])
                # direc = direct_current + self.normalize_angle(direct_next-direct_current)
            elif i==len(waypoints_copy)-1:
                direct_current = self.calculate_angle(waypoints[i-1],waypoints[i])
                direct_next = self.calculate_angle(waypoints[i],waypoints[1])
                # direc = direct_current + self.normalize_angle(direct_next-direct_current)
           
            else :
                direct_current = self.calculate_angle(waypoints[i-1],waypoints[i])
                direct_next = self.calculate_angle(waypoints[i],waypoints[i+1])
                # direc = direct_current + 0.8*self.normalize_angle(direct_next-direct_current)
           
            way_pose = pose_from_xytheta(waypoints[i][0], waypoints[i][1], direct_next)
            waypoints_copy[i] = PoseStamped()
            waypoints_copy[i].header.frame_id = 'map'
            waypoints_copy[i].header.stamp = self.navigator.get_clock().now().to_msg()
            
            waypoints_copy[i].pose = way_pose
            
        self.navigator.followWaypoints(waypoints_copy)
class CostmapCallback(Node):
    def __init__(self):
        super().__init__('costmap_callback')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/global_costmap',  
            self.costmap_callback,
            10)
    
    def costmap_callback(self, msg):
        try:
            costmap_in_base_link = self.tf_buffer.transform(msg, 'base_link', timeout=rclpy.time.Duration(1))
            # Process the costmap data here (e.g., analyze obstacles)
            velocity_scale = self.velocity_scale(costmap_in_base_link)
        
            # Set the robot's velocity based on the calculated scale
            self.adjust_robot_velocity(velocity_scale)
                
        except Exception as e:
            self.get_logger().error(f"Costmap callback error: {str(e)}")
    
    
    def velocity_scale(self,costmap):
        max_cost = self.get_max_cost_in_vicinity(costmap)
    
        # Define a scaling factor based on the maximum cost (adjust as needed)
        max_cost_threshold = 40  #
        max_velocity_scale = 0.1  #
            
            # Calculate the velocity scale
        velocity_scale = max(1.0 - max_cost / max_cost_threshold, max_velocity_scale)
            
        return velocity_scale
    def get_max_cost_in_vicinity(self, costmap):
    # Define the region around the robot to consider
        robot_x = 0  
        robot_y = 0 
        vicinity_radius = 0.8z  # Define the radius (adjust as needed)

        # Calculate the grid indices of the robot's position in the costmap
        robot_x_index = int((robot_x - costmap.info.origin.position.x) / costmap.info.resolution)
        robot_y_index = int((robot_y - costmap.info.origin.position.y) / costmap.info.resolution)

        # Calculate the number of cells to consider around the robot
        cells_to_check = int(vicinity_radius / costmap.info.resolution)

            # Initialize the maximum cost value
        max_cost = 0

            # Loop through the cells in the vicinity of the robot
        for dx in range(-cells_to_check, cells_to_check + 1):
            for dy in range(-cells_to_check, cells_to_check + 1):
                x_index = robot_x_index + dx
                y_index = robot_y_index + dy
                # Check if the indices are within the costmap boundaries
                if 0 <= x_index < costmap.info.width and 0 <= y_index < costmap.info.height:
                    cell_cost = costmap.data[x_index + y_index * costmap.info.width]

                    # Update the maximum cost value if needed
                    max_cost = max(max_cost, cell_cost)

        return max_cost
    def adjust_robot_velocity(self, velocity_scale):
        velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

         # Create a Twist message to set linear velocity
        twist = Twist()
        twist.linear.x = velocity_scale  # Set the linear velocity

        # Publish the Twist message to control the robot's velocity
        velocity_publisher.publish(twist)

def pose_from_xytheta(x, y, theta):
        # negative theta: turn clockwise
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        q = quaternion_from_euler(0, 0, theta)
        pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return pose
def main():

    rclpy.init()
    print("start")
    navigator = BasicNavigator()
    CostmapCallback()
    navi = Navigation()
    
    pose = navi.get_pos()
    if pose == None:
        time.sleep(0.5)
        pose = [0.0,0.0]
    # Set our demo's initial pose (0,0,0)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose = pose_from_xytheta(pose[0], pose[1], 0.0)
    navigator.setInitialPose(initial_pose)
    pos_init = [initial_pose.pose.position.x,initial_pose.pose.position.y,0.0]

    waypoints = navi.path_planning(pos_init)
    # Wait for navigation to fully activate, since autostarting nav2s
    navigator.waitUntilNav2Active()
    navi.navigate(waypoints)

    i = 0
    while not navigator.isTaskComplete():
        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()

            # Some navigation request change to demo preemption
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                waypoints = navi.path_planning()
                navi.navigate(waypoints)
                

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    navi.destroy_node()
    rclpy.shutdown()
    exit(0)
if __name__ == '__main__':
    main()
