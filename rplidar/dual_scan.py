from pyrplidar import PyRPlidar
import time


def scan():
    lidar1 = PyRPlidar()
    lidar2 = PyRPlidar()

    try:
        lidar1.connect(port="COM7", baudrate=1000000, timeout=5)
        lidar1.set_motor_pwm(700)

        lidar2.connect(port="COM8", baudrate=1000000, timeout=5)
        lidar2.set_motor_pwm(700)
        time.sleep(2)

        print("Starting scan...")
        scan_generator1 = lidar1.start_scan_express(0)()
        scan_generator2 = lidar2.start_scan_express(0)()

        while True:
            scan1 = next(scan_generator1)
            scan2 = next(scan_generator2)
            #print(scan.quality, scan.angle, scan.distance)
            print(f"LiDAR_1: {scan1}")
            print(f"LiDAR_2: {scan2}")


    except KeyboardInterrupt:
        print("Stopping scan...")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        lidar1.set_motor_pwm(0)
        lidar1.stop()
        lidar1.disconnect()

        lidar2.set_motor_pwm(0)
        lidar2.stop()
        lidar2.disconnect()

        time.sleep(1)

if __name__ == '__main__':
    scan()
