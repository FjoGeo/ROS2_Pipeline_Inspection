# Install IMU on Linux
---

connect and check if it's displayed

    sudo dmesg | grep tty

or 

    ls /dev/tty*

if not connected:

    sudo apt remove brltty

then unplug and replug it