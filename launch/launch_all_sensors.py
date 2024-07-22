from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rp_test',
            executable='talker',
            name='rplidar_talker'
        ),
        Node(
            package='witmotion_imu',
            executable='talker',
            name='witmotion_imu_talker'
        ),

        Node(
            package='my_realsense',
            executable='talker',
            name='my_realsense_talker'
        ),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', 'bagfile_for_sensor', 
                 'lidar/quality1',  'lidar/angle1', 'lidar/distance1', 
                 'lidar/quality2',  'lidar/angle2', 'lidar/distance2', 
                 '/serial_data/AccX', '/serial_data/AccY', '/serial_data/AccZ', 
                 '/serial_data/AngX', '/serial_data/AngY', '/serial_data/AngZ',
                 '/serial_data/AsX', '/serial_data/AsY', '/serial_data/AsZ',
                 '/serial_data/HX', '/serial_data/HY', '/serial_data/HZ',           
                 '/realsense1/rgb', '/realsense1/depth', '/realsense1/pointcloud',
                 '/realsense2/rgb', '/realsense2/depth', '/realsense2/pointcloud',],
            output='screen'
        )
 
    ])
