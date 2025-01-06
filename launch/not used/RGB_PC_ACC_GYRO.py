from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import datetime

def generate_launch_description():
    current_time = datetime.datetime.now()

    return LaunchDescription([
        

        Node(
            package='my_realsense',
            executable='talker_rgb_pc',
            name='RGB_PC_TALKER'
        ),


        Node(
            package='my_realsense',
            executable='talker_acc_gyro',
            name='GYRO_AND_ACC_TALKER'
        ),

       

        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', f'bagfile_{current_time}',          
                 '/realsense1/rgb',  '/realsense1/pointcloud', '/realsense1/accel', '/realsense1/gyro',
                 '/realsense2/rgb', '/realsense2/pointcloud', '/realsense2/accel', '/realsense2/gyro'
                ],
            output='screen'
        )
 
    ])
