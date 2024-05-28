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

- go /src


      ros2 pkg create --build-type ament_python <name_of_package>

- create subscriber and publisher
- update setup.py
- update package.xml
- update config.cfg 
