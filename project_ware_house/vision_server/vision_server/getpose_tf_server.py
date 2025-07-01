import rclpy
from rclpy.node import Node
from vision_interface.srv import GetPose
from vision_interface.msg import SinglePose 
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