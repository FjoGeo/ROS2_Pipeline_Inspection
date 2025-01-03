# scna with rplidar library and not pyrplidar

from rplidar import RPLidar
import time

def scan():
    lidar = RPLidar(port="/dev/ttyUSB0", baudrate=1000000, timeout=5)

    try:
        lidar.connect()
        time.sleep(2)

        print("Starting scan...")
        scan_generator = lidar.iter_scans()

        qualities, angles, ranges = [], [], []

        while True:
            scan = next(scan_generator)
            # print("Raw scan data:", scan)

            # Unpack tuples into separate lists
            for quality, angle, distance in scan:
                qualities.append(quality)
                angles.append(angle)
                ranges.append(distance)

                print(f"Qualities: {quality}, Angles: {angle}, Ranges: {distance}")

    except KeyboardInterrupt:
        print("Stopping scan...")
        lidar.stop()
        lidar.disconnect()
        time.sleep(1)
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        lidar.stop()
        lidar.disconnect()
        time.sleep(1)

if __name__ == '__main__':
    scan()
