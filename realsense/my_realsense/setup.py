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
            'talker = my_realsense.talker:main',
            'listener_rgb = my_realsense.sub_rgb:main',
            'listener_depth = my_realsense.sub_depth:main',
            'listener_pointcloud = my_realsense.sub_pc:main',
        ],
    },
)
