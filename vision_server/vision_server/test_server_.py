#!/urs/bin/env python3

import rclpy
from rclpy.node import Node
import math
from vision_interface.srv import GetPose

class GetPose_Server(Node):
    def __init__(self):
        super().__init__("GetPose_Server")
        self.server_ = self.create_service(GetPose, "get_pose_service", self.get_pose_service_callback)
        self.get_logger().info("Get pose Service server node has been created")
    
    def get_pose_service_callback(self, request, response): 


        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        
        # Simulate some pose data (you would replace this with actual pose retrieval)
        x = math.sin(current_time * 0.1) * 2.0
        y = math.cos(current_time * 0.1) * 2.0
        z = 1.0
        # roll = 0.0
        # pitch = 0.0
        # yaw = current_time * 0.05
        
        # Return pose as array of floats
        # response.pose_data = [x, y, z, roll, pitch, yaw]
        response.pose_x = 1.0
        response.pose_y = 1.0
        response.pose_z = 1.0
        
        self.get_logger().info(f'Returning pose: {response}')
        return response 


def main(args = None):
    rclpy.init(args=args)
    node = GetPose_Server()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=='__main__':
    main()