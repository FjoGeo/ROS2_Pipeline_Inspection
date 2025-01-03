from rplidar import RPLidar
import time


def scan():
    lidar1 = RPLidar(port="/dev/ttyUSB0", baudrate=1000000, timeout=5)
    lidar2 = RPLidar(port="/dev/ttyUSB1", baudrate=1000000, timeout=5)

    try:
        lidar1.connect()
        lidar2.connect()
        time.sleep(2)

        print("Starting scan...")
        scan_generator1 = lidar1.iter_scans()
        scan_generator2 = lidar2.iter_scans()

        qualities1, angles1, ranges1 = [], [], []
        qualities2, angles2, ranges2 = [], [], []

        while True:
            scan1 = next(scan_generator1)
            scan2 = next(scan_generator2)

            for quality, angle, distance in scan:
                qualities1.append(quality)
                angles1.append(angle)
                ranges1.append(distance)

                print(f"Qualities: {quality}, Angles: {angle}, Ranges: {distance}")

            for quality, angle, distance in scan:
                qualities2.append(quality)
                angles2.append(angle)
                ranges2.append(distance)

                print(f"Qualities: {quality}, Angles: {angle}, Ranges: {distance}")



    except KeyboardInterrupt:
        print("Stopping scan...")
        try:
            lidar1.stop()
            lidar1.disconnect()
            lidar2.stop()
            lidar2.disconnect()
            time.sleep(1)
        except:
            pass
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        try:
            lidar1.stop()
            lidar1.disconnect()
            lidar2.stop()
            lidar2.disconnect()
            time.sleep(1)
        except:
            pass

if __name__ == '__main__':
    scan()
