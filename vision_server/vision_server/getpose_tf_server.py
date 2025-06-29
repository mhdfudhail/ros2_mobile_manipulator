# # ============================================================================
# # Service Server Implementation with TF
# # Save as: getpose_server.py

# import rclpy
# from rclpy.node import Node
# from vision_interface.srv import GetPose
# from tf2_ros import TransformListener, Buffer
# from tf2_ros.transform_listener import TransformException
# import tf2_geometry_msgs
# from geometry_msgs.msg import TransformStamped, PoseStamped
# import threading
# from rclpy.time import Time

# class GetPoseServer(Node):
#     def __init__(self):
#         super().__init__('getpose_server')
        
#         # Service
#         self.srv = self.create_service(GetPose, 'get_pose_service', self.get_pose_callback)
        
#         # TF2 setup
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)
        
#         # Configuration - you can modify these frame names as needed
#         self.target_frame = 'base_link'  # Frame to get pose relative to
#         self.source_frame = 'Red'  # Frame of the object you want to track
        
#         # Thread safety
#         self.tf_lock = threading.Lock()
        
#         self.get_logger().info('GetPose service server ready.')
#         self.get_logger().info(f'Waiting for TF between {self.source_frame} and {self.target_frame}...')

#     def get_tf_pose(self, target_frame, source_frame, timeout_sec=1.0):
#         """
#         Get pose of source_frame relative to target_frame using TF
#         Returns: (x, y, z, qx, qy, qz, qw) or None if transform not available
#         """
#         try:
#             # Get the most recent transform
#             now = self.get_clock().now()
            
#             # Wait for transform to be available
#             if not self.tf_buffer.can_transform(target_frame, source_frame, now, 
#                                                 rclpy.duration.Duration(seconds=timeout_sec)):
#                 self.get_logger().warn(f'Transform from {source_frame} to {target_frame} not available')
#                 return None
            
#             # Get the transform
#             transform = self.tf_buffer.lookup_transform(
#                 target_frame, source_frame, now,
#                 rclpy.duration.Duration(seconds=timeout_sec)
#             )
            
#             # Extract position and orientation
#             trans = transform.transform.translation
#             rot = transform.transform.rotation
            
#             return (float(trans.x), float(trans.y), float(trans.z),
#                    float(rot.x), float(rot.y), float(rot.z), float(rot.w))
            
#         except TransformException as e:
#             self.get_logger().error(f'TF lookup failed: {str(e)}')
#             return None
#         except Exception as e:
#             self.get_logger().error(f'Error getting TF pose: {str(e)}')
#             return None

#     def get_pose_callback(self, request, response):
#         """
#         Service callback that returns TF pose
#         """
#         self.get_logger().info(f'Received GetPose request with req={request.req}')
        
#         # Default response
#         response.pose_x = 0.0
#         response.pose_y = 0.0
#         response.pose_z = 0.0
        
#         if not request.req:
#             self.get_logger().info('Request flag is False, returning zero pose')
#             return response
        
#         # Get TF pose
#         with self.tf_lock:
#             pose_data = self.get_tf_pose(self.target_frame, self.source_frame)
        
#         if pose_data is None:
#             self.get_logger().warn(f'Could not get TF pose from {self.source_frame} to {self.target_frame}')
#             return response
        
#         try:
#             # Extract position (ignoring orientation for now, as original service only returned x,y,z)
#             x, y, z, qx, qy, qz, qw = pose_data
            
#             # Set response
#             response.pose_x = x
#             response.pose_y = y
#             response.pose_z = z
            
#             self.get_logger().info(
#                 f'Returning TF pose: x={response.pose_x:.3f}, y={response.pose_y:.3f}, z={response.pose_z:.3f}'
#             )
            
#         except Exception as e:
#             self.get_logger().error(f'Error processing TF data: {str(e)}')
        
#         return response

#     def set_frame_names(self, target_frame, source_frame):
#         """
#         Method to change the frame names if needed
#         """
#         self.target_frame = target_frame
#         self.source_frame = source_frame
#         self.get_logger().info(f'Updated frames: target={target_frame}, source={source_frame}')

# def main(args=None):
#     rclpy.init(args=args)
#     getpose_server = GetPoseServer()
    
#     # Optional: Set custom frame names
#     # getpose_server.set_frame_names('world', 'detected_object')
    
#     try:
#         rclpy.spin(getpose_server)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         getpose_server.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
# ============================================================================
# Service Server Implementation with Single Pose Topic
# Save as: getpose_server.py

import rclpy
from rclpy.node import Node
from vision_interface.srv import GetPose
from vision_interface.msg import SinglePose  # Assuming this is your custom message
import threading

class GetPoseServer(Node):
    def __init__(self):
        super().__init__('getpose_server')
        
        # Service
        self.srv = self.create_service(GetPose, 'get_pose_service', self.get_pose_callback)
        
        # Subscriber
        self.single_pose_sub = self.create_subscription(
            SinglePose, '/single_pose', self.single_pose_callback, 10)
        
        # Data storage
        self.current_pose = None
        self.pose_lock = threading.Lock()
        
        self.get_logger().info('GetPose service server ready.')
        self.get_logger().info('Waiting for pose data from /single_pose topic...')

    def single_pose_callback(self, msg):
        """Store the latest pose data"""
        with self.pose_lock:
            self.current_pose = msg
            self.get_logger().debug(f'Received pose for object "{msg.object}": x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}')

    def get_pose_callback(self, request, response):
        """
        Service callback that returns the latest pose from /single_pose topic
        """
        self.get_logger().info(f'Received GetPose request with req={request.req}')
        
        # Default response
        response.pose_x = 0.0
        response.pose_y = 0.0
        response.pose_z = 0.0
        
        if not request.req:
            self.get_logger().info('Request flag is False, returning zero pose')
            return response
        
        # Get current pose data
        with self.pose_lock:
            current_pose = self.current_pose
            
        if current_pose is None:
            self.get_logger().warn('No pose data available from /single_pose topic')
            return response
        
        try:
            # Set response from the received pose data
            response.pose_x = float(current_pose.x)
            response.pose_y = float(current_pose.y)
            response.pose_z = float(current_pose.z)
            
            self.get_logger().info(
                f'Returning pose for object "{current_pose.object}": x={response.pose_x:.3f}, y={response.pose_y:.3f}, z={response.pose_z:.3f}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error processing pose data: {str(e)}')
        
        return response

def main(args=None):
    rclpy.init(args=args)
    getpose_server = GetPoseServer()
    
    try:
        rclpy.spin(getpose_server)
    except KeyboardInterrupt:
        pass
    finally:
        getpose_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()