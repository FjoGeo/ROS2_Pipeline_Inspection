from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import datetime

def generate_launch_description():
    current_time = datetime.datetime.now()

    return LaunchDescription([
        Node(
            package='witmotion_imu',
            executable='talker',
            name='witmotion_imu_talker'
        ),

        Node(
            package='my_realsense',
            executable='talker_rgb',
            name='my_realsense_talker'
        ),


        Node(
            package='my_realsense',
            executable='talker_acc_gyro',
            name='my_realsense_talker'
        ),


        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', f'bagfile_{current_time}',        
                 '/serial_data/AccX', '/serial_data/AccY', '/serial_data/AccZ', 
                 '/serial_data/AngX', '/serial_data/AngY', '/serial_data/AngZ',
                 '/serial_data/AsX', '/serial_data/AsY', '/serial_data/AsZ',
                 '/serial_data/HX', '/serial_data/HY', '/serial_data/HZ',           
                 '/realsense1/rgb',  '/realsense1/accel', '/realsense1/gyro',
                 '/realsense2/rgb',  '/realsense2/accel', '/realsense2/gyro'         
                ],
            output='screen'
        )
 
    ])
