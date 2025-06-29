#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vision_interface.srv import GetPose
from functools import partial

class SumClientNode(Node):
    def __init__(self):
        super().__init__('sum_client_node')
        self.get_logger().info('Sum Client Python node has been created')

        # declare parameters for AddTwoInts
        a_ = 4
        b_ = 7
        
        self.call_sum_server()
    
    def call_sum_server(self):
        client = self.create_client(GetPose, '/get_pose_service')
        while not client.wait_for_service(1.0):
            self.get_logger().info('Waiting for the Server...')

        # create request
        request = GetPose.Request()
        request.req = True
        # request.b = b


        #send request asynchronously
        future = client.call_async(request)
        future.add_done_callback(partial(self.sum_service_callback))

    def sum_service_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'{response.pose_x} - {response.pose_y} - {response.pose_z}')
        except Exception as e:
            self.get_logger().info(f'Service call failed {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SumClientNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=='__main__':
    main()