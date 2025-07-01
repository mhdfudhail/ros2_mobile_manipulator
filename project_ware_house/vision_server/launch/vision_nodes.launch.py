from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    
    object_locator_server_node = Node(
        package='vision_server',
        executable='object_locator_server',
        name='object_locator_server',
    )

    pose_pcl_server_node = Node(
        package='vision_server',
        executable='get_pose_tf_server',
        name='get_pose_tf_server'
    )
    object_pose_node = Node(
        package='vision_server',
        executable='object_tf',
        name='object_tf'
    )


    return LaunchDescription([
        object_locator_server_node,
        pose_pcl_server_node,
        object_pose_node
    ])
