from setuptools import find_packages, setup



package_name = 'my_realsense'



setup(

    name=package_name,

    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[

        ('share/ament_index/resource_index/packages',

            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

    ],

    install_requires=['setuptools','rclpy', 'sensor_msgs', 'cv_bridge', 'numpy', 'pyrealsense2'],

    zip_safe=True,

    maintainer='user',

    maintainer_email='user@todo.todo',

    description='TODO: Package description',

    license='TODO: License declaration',

    tests_require=['pytest'],

    entry_points={

        'console_scripts': [

            'talker_depth = my_realsense.talker_depth:main',

            'talker_rgb = my_realsense.talker_rgb:main',

            'talker_pc = my_realsense.talker_pc:main',

            'talker_acc_gyro = my_realsense.talker_acc_gyro:main',

            'talker_IR = my_realsense.talker_IR:main',

            'sub_rgb = my_realsense.sub_rgb:main',

            'sub_depth = my_realsense.sub_depth:main',

            'listener_pointcloud = my_realsense.sub_pc:main',

        ],

    },

)
