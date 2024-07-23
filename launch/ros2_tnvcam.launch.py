from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tnvcam_ros2',
            executable='nvv4l2_node',
            name='front_camera_node',
            parameters=[{'camera_id': 0}],    
            remappings=[
                ('/image_raw', '/image_front')
            ],
        ),
        Node(
            package='tnvcam_ros2',
            executable='nvv4l2_node',
            name='front_right_camera_node',
            parameters=[{'camera_id': 1}],    
            remappings=[
                ('/image_raw', '/image_front_right')
            ],
        ),
        Node(
            package='tnvcam_ros2',
            executable='nvv4l2_node',
            name='front_left_camera_node',
            parameters=[{'camera_id': 2}],    
            remappings=[
                ('/image_raw', '/image_front_left')
            ],
        ),
        Node(
            package='tnvcam_ros2',
            executable='nvv4l2_node',
            name='rear_right_camera_node',
            parameters=[{'camera_id': 3}],    
            remappings=[
                ('/image_raw', '/image_rear_right')
            ],
        ),
        Node(
            package='tnvcam_ros2',
            executable='nvv4l2_node',
            name='rear_left_camera_node',
            parameters=[{'camera_id': 4}],    
            remappings=[
                ('/image_raw', '/image_rear_left')
            ],
        ),
        Node(
            package='tnvcam_ros2',
            executable='nvv4l2_node',
            name='front_down_camera_node',
            parameters=[{'camera_id': 5}],    
            remappings=[
                ('/image_raw', '/image_front_down')
            ],
        ),
    ])
