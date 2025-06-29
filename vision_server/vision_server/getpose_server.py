
import rclpy
from rclpy.node import Node
from vision_interface.srv import GetPose
from vision_interface.msg import ObjectPose  # Your custom message
from sensor_msgs.msg import PointCloud2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
import threading

class GetPoseServer(Node):
    def __init__(self):
        super().__init__('getpose_server')
        
        # Service
        self.srv = self.create_service(GetPose, 'get_pose_service', self.get_pose_callback)
        
        # Subscribers
        self.object_pose_sub = self.create_subscription(
            ObjectPose, '/pose_publisher', self.object_pose_callback, 10)
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/camera/points', self.pointcloud_callback, 10)
        
        # Data storage
        self.current_object_pose = None
        self.current_pointcloud = None
        self.object_pose_lock = threading.Lock()
        self.pointcloud_lock = threading.Lock()
        
        self.get_logger().info('GetPose service server ready.')
        self.get_logger().info('Waiting for object pose and point cloud data...')

    def object_pose_callback(self, msg):
        """Store the latest object pose data"""
        with self.object_pose_lock:
            self.current_object_pose = msg
            self.get_logger().debug(f'Received object pose: {msg.object_1} at pixel ({msg.pixel1_x}, {msg.pixel1_y})')

    def pointcloud_callback(self, msg):
        """Store the latest point cloud"""
        with self.pointcloud_lock:
            self.current_pointcloud = msg

    def get_3d_point_from_pixel(self, pixel_x, pixel_y, pointcloud_msg):
        """
        Get 3D coordinates from pixel coordinates using point cloud
        Returns: (x, y, z) in meters or None if invalid
        """
        try:
            # Get point cloud data as generator
            points = pc2.read_points(pointcloud_msg, skip_nans=True, 
                                   field_names=("x", "y", "z"))
            
            # Convert to list for indexing (this might be memory intensive for large clouds)
            points_list = list(points)
            
            # Calculate index in point cloud array
            # Point clouds are typically organized as width x height
            width = pointcloud_msg.width
            height = pointcloud_msg.height
            
            # Check bounds
            if pixel_x >= width or pixel_y >= height or pixel_x < 0 or pixel_y < 0:
                self.get_logger().warn(f"Pixel coordinates ({pixel_x}, {pixel_y}) out of bounds")
                return None
            
            # Calculate index (row-major order)
            index = pixel_y * width + pixel_x
            
            if index < len(points_list):
                point = points_list[index]
                x, y, z = point
                
                # Check for valid point (not NaN or infinity)
                if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                    return (float(x), float(y), float(z))
                else:
                    self.get_logger().warn(f"Invalid 3D point at pixel ({pixel_x}, {pixel_y})")
                    return None
            else:
                self.get_logger().warn(f"Point cloud index {index} out of range")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Error getting 3D point: {str(e)}")
            return None

    def get_pose_callback(self, request, response):
        """
        Service callback that uses object pose data and returns 3D position
        """
        self.get_logger().info(f'Received GetPose request with req={request.req}')
        
        # Default response
        response.pose_x = 0.0
        response.pose_y = 0.0
        response.pose_z = 0.0
        
        if not request.req:
            self.get_logger().info('Request flag is False, returning zero pose')
            return response
        
        # Check if we have current data
        with self.object_pose_lock:
            current_object_pose = self.current_object_pose
        with self.pointcloud_lock:
            current_pointcloud = self.current_pointcloud
            
        if current_object_pose is None:
            self.get_logger().warn('No object pose data available from /pose_object topic')
            return response
            
        if current_pointcloud is None:
            self.get_logger().warn('No point cloud data available from /camera/points topic')
            return response
        
        try:
            # Get pixel coordinates from object pose message
            pixel_x = int(current_object_pose.pixel1_x)
            pixel_y = int(current_object_pose.pixel1_y)
            object_name = current_object_pose.object_1
            
            self.get_logger().info(f'Processing object "{object_name}" at pixel ({pixel_x}, {pixel_y})')
            
            # Get 3D coordinates from point cloud
            point_3d = self.get_3d_point_from_pixel(pixel_x, pixel_y, current_pointcloud)
            
            if point_3d is None:
                self.get_logger().warn('Could not get valid 3D coordinates for detected object')
                return response
            
            # Set response
            response.pose_x, response.pose_y, response.pose_z = point_3d
            
            self.get_logger().info(
                f'Returning 3D pose for "{object_name}": x={response.pose_x:.3f}, y={response.pose_y:.3f}, z={response.pose_z:.3f}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error processing request: {str(e)}')
        
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
