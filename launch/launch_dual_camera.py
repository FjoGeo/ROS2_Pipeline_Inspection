from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_realsense',
            executable='talker_pc',
            name='my_realsense_talker'
        ),
        Node(
            package='my_realsense',
            executable='talker_acc_gyro',
            name='my_realsense_talker'
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', 'bagfile_for_sensor',          
                  '/realsense1/pointcloud', '/realsense1/accel', '/realsense1/gyro',
                 '/realsense2/pointcloud', '/realsense2/accel', '/realsense2/gyro'
                ],
            output='screen'
        )
 
    ])
