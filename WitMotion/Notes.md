# Install IMU on Linux
---

connect and check if it's displayed

    sudo dmesg | grep tty

or 

    ls /dev/tty*

if not connected:

    sudo apt remove brltty

then unplug and replug it

check rights:

    ls -l /dev/ttyUSB0

# Install PySerial
---



- Download from github.com/pyserial/pyserial/releases
- Install (Conda does not work / needs workaround)

# Install a ROS2 node
---

(Tutorial:)[https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html]

- go to /src


      ros2 pkg create --build-type ament_python <name_of_package>

- create subscriber and publisher
- update setup.py
- update package.xml
- update config.cfg
- go back to workspace directory

      colcon build --packages-select <name_of_pkg>

- source

          source ~/<your_ros_directory>/install/local_setup.bas
