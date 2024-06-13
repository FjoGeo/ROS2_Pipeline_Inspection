from pyrplidar import PyRPlidar
import time



def scan():
    lidar = PyRPlidar()

    try:
        lidar.connect(port="COM7", baudrate=1000000, timeout=10)

        info = lidar.get_info()
        print("info :", info)


        lidar.set_motor_pwm(700)
        time.sleep(2)

        print(lidar.get_health())

        print("Starting scan...")
        scan_generator = lidar.start_scan_express(0)()

        while True:
            scan = next(scan_generator)
            #print(scan.quality, scan.angle, scan.distance)
            print(scan)


    except KeyboardInterrupt:
        print("Stopping scan...")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        lidar.set_motor_pwm(0)
        lidar.stop()
        lidar.disconnect()
        time.sleep(1)

if __name__ == '__main__':
    scan()
