from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rp_test',
            executable='talker',
            name='talker_node'
        ),
        Node(
            package='witmotion_imu',
            executable='talker',
            name='talker_node2'
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', 'my_bag', '/lidar_scan /serial_data'],
            output='screen'
        )
 
    ])