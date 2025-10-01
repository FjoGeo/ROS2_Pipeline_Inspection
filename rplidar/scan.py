from rplidar import RPLidar
import time

def scan():
    lidar = RPLidar(port="/dev/ttyUSB0", baudrate=1000000, timeout=5)

    try:
        lidar.connect()
        time.sleep(2)

        print("Starting scan...")
        scan_generator = lidar.iter_measurments(max_buf_meas=500)

        while True:
            # Print each quality, angle, distance separately
            for new_scan, quality, angle, distance in scan_generator:
                print(f"Quality: {quality}, Angle: {angle}°, Distance: {distance} mm")

    except KeyboardInterrupt:
        print("Stopping scan...")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        lidar.stop()
        lidar.disconnect()
        time.sleep(1)

if __name__ == '__main__':
    scan()