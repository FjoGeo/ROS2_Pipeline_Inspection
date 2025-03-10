from setuptools import find_packages, setup

package_name = 'laser_scanner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','rclpy', 'sensor_msgs', 'ctypes'],
    zip_safe=True,
    maintainer='intelnuc',
    maintainer_email='fjodorow90@googlemail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'talker = laser_scanner.talker:main',
        ],
    },
)
