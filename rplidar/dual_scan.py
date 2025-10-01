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
        scan_generator1 = lidar1.iter_measurments(max_buf_meas=500)
        scan_generator2 = lidar2.iter_measurments(max_buf_meas=500)


        while True:

            for new_scan, quality, angle, distance in scan_generator1:
                print(f"Quality: {quality}, Angle: {angle}°, Distance: {distance} mm")

            for qnew_scan, quality, angle, distance in scan_generator2:
                print(f"Quality: {quality}, Angle: {angle}°, Distance: {distance} mm")

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
