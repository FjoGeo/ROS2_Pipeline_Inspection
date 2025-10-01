from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import datetime

def generate_launch_description():
    current_time = datetime.datetime.now()

    return LaunchDescription([


        Node(
            package='laser_scanner',
            executable='talker',
            name='scanControl30xx'
        ),


        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', f'LASER_bagfile_{current_time}',              
                    '/laser_scan'
                ],
            output='screen'
        )
    ])
