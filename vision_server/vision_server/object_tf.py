import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from vision_interface.msg import SinglePose
from geometry_msgs.msg import TransformStamped, PointStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

# Replace with your actual package and message definition
from vision_interface.msg import ObjectPose  # <-- Update this line

class ObjectPointTFPublisher(Node):
    def __init__(self):
        super().__init__('object_point_tf_publisher')

        self.latest_pointcloud = None

        # TF broadcaster and listener
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub_pose = self.create_publisher(SinglePose, "single_pose",10) #to publish processed image


        # Subscriptions
        self.create_subscription(PointCloud2, '/camera/points', self.pc_callback, 10)
        self.create_subscription(ObjectPose, '/pose_publisher', self.pose_callback, 10)

    def pc_callback(self, msg):
        self.latest_pointcloud = msg

    def pose_callback(self, msg):
        if self.latest_pointcloud is None:
            self.get_logger().warn("No point cloud received yet.")
            return

        u = msg.pixel2_x
        v = msg.pixel2_y
        width = self.latest_pointcloud.width
        height = self.latest_pointcloud.height

        if not (0 <= u < width) or not (0 <= v < height):
            self.get_logger().warn("Pixel coordinates out of bounds.")
            return

        index = v * width + u

        try:
            points = list(pc2.read_points(self.latest_pointcloud, field_names=("x", "y", "z"), skip_nans=False))

            if index >= len(points):
                self.get_logger().warn("Point index out of bounds.")
                return

            x, y, z = points[index]

            if any(map(np.isnan, [x, y, z])):
                self.get_logger().warn("Invalid point at pixel.")
                return

            # Wrap point as PointStamped in camera frame
            camera_frame = self.latest_pointcloud.header.frame_id
            point_stamped = PointStamped()
            point_stamped.header.stamp = self.get_clock().now().to_msg()
            point_stamped.header.frame_id = camera_frame
            point_stamped.point.x = float(x)
            point_stamped.point.y = float(y)
            point_stamped.point.z = float(z)

            # Transform to base_link
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link',
                    camera_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                transformed_point = do_transform_point(point_stamped, transform)
                x_base = transformed_point.point.x
                y_base = transformed_point.point.y
                z_base = transformed_point.point.z
            except Exception as e:
                self.get_logger().warn(f"TF transform failed: {e}")
                return

            # Publish as TF
            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = 'base_link'
            tf_msg.child_frame_id = msg.object_2
            tf_msg.transform.translation.x = x_base
            tf_msg.transform.translation.y = y_base
            tf_msg.transform.translation.z = z_base
            tf_msg.transform.rotation.x = 0.0
            tf_msg.transform.rotation.y = 0.0
            tf_msg.transform.rotation.z = 0.0
            tf_msg.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(tf_msg)
            single_pose = SinglePose()
            single_pose.object = msg.object_2
            single_pose.x = x_base
            single_pose.y = y_base
            single_pose.z = z_base
            self.pub_pose.publish(single_pose)
            self.get_logger().info(f"Published TF for {msg.object_2} at ({x_base:.2f}, {y_base:.2f}, {z_base:.2f})")

        except Exception as e:
            self.get_logger().error(f"Error processing point: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectPointTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
