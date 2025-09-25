#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathThroughPoses
from geometry_msgs.msg import PoseStamped, PoseArray
from std_srvs.srv import Trigger
import math

class PathRequestNode(Node):
    def __init__(self):
        super().__init__('path_request_node')

        self.action_client = ActionClient(
            self,
            ComputePathThroughPoses,
            '/compute_path_through_poses'
        )

        self.trigger_service = self.create_service(
            Trigger,
            'trigger_path_request',
            self.trigger_callback
        )

        # Publisher for visualization
        self.poses_pub = self.create_publisher(PoseArray, '/goal_poses', 10)

        # Timer to continuously publish poses for visualization
        self.create_timer(1.0, self.publish_poses)

        # Store pose data - (x, y, z, qx, qy, qz, qw)
        self.start_pose_data = (31.057, 19.871, 0.0, 0.0, 0.0, 0.707, 0.707)
        self.intermediate_pose_data = (7.73, 14.00, 0.0, 0.0, 0.0, 0.23, 0.97)
        self.end_pose_data = (10.42, 15.33, 0.0, 0.0, 0.0, 0.23, 0.97)

        self.get_logger().info('Path Request Node initialized')
        self.get_logger().info('Call /trigger_path_request service to send path request')

    def create_pose_stamped(self, pose_data):
        """Helper to create PoseStamped from pose data tuple"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = pose_data[0]
        pose.pose.position.y = pose_data[1]
        pose.pose.position.z = pose_data[2]
        pose.pose.orientation.x = pose_data[3]
        pose.pose.orientation.y = pose_data[4]
        pose.pose.orientation.z = pose_data[5]
        pose.pose.orientation.w = pose_data[6]
        return pose

    def publish_poses(self):
        """Continuously publish the three goal poses for visualization"""
        pose_array = PoseArray()
        pose_array.header.frame_id = 'map'
        pose_array.header.stamp = self.get_clock().now().to_msg()

        start = self.create_pose_stamped(self.start_pose_data)
        intermediate = self.create_pose_stamped(self.intermediate_pose_data)
        end = self.create_pose_stamped(self.end_pose_data)

        pose_array.poses = [start.pose, intermediate.pose, end.pose]
        self.poses_pub.publish(pose_array)

    def trigger_callback(self, request, response):
        self.get_logger().info('Trigger received, sending path request')

        # Create goal with start and end poses
        goal = ComputePathThroughPoses.Goal()

        # Create poses from stored data
        start_pose = self.create_pose_stamped(self.start_pose_data)
        intermediate_pose = self.create_pose_stamped(self.intermediate_pose_data)
        end_pose = self.create_pose_stamped(self.end_pose_data)

        # Set start pose
        goal.start = start_pose

        # Set goals (intermediate and end poses)
        goal.goals = [intermediate_pose, end_pose]

        # Set planner ID (use the first one from params)
        goal.planner_id = 'GridBased1'

        # Use global costmap
        goal.use_start = True

        self.get_logger().info(f'Sending path request:')
        self.get_logger().info(f'  Start: ({start_pose.pose.position.x}, {start_pose.pose.position.y}) facing +X')
        self.get_logger().info(f'  Intermediate: ({intermediate_pose.pose.position.x}, {intermediate_pose.pose.position.y}) facing +Y')
        self.get_logger().info(f'  End: ({end_pose.pose.position.x}, {end_pose.pose.position.y}) facing +Y')

        # Wait for action server
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            response.success = False
            response.message = 'Action server not available'
            return response

        # Send goal
        future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.done():
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info('Goal accepted')

                # Wait for result
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)

                if result_future.done():
                    result = result_future.result()
                    if result:
                        self.get_logger().info(f'Path computed successfully with {len(result.result.path.poses)} poses')
                        response.success = True
                        response.message = f'Path computed with {len(result.result.path.poses)} poses'
                    else:
                        self.get_logger().error('Failed to get result')
                        response.success = False
                        response.message = 'Failed to get result'
                else:
                    self.get_logger().error('Timeout waiting for result')
                    response.success = False
                    response.message = 'Timeout waiting for result'
            else:
                self.get_logger().error('Goal rejected')
                response.success = False
                response.message = 'Goal rejected'
        else:
            self.get_logger().error('Failed to send goal')
            response.success = False
            response.message = 'Failed to send goal'

        return response

def main(args=None):
    rclpy.init(args=args)
    node = PathRequestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()