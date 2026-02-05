from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='zed_cpu_ros2',
            executable='zed_node_ros2',
            name='zed_node',
            output='screen',
            emulate_tty=True
        )
        # Node(
        #     package='zed_cpu_ros2',
        #     executable='zed2i_rectification_node',
        #     name='image_rect_node',
        #     output='log'
        # )
    ])
