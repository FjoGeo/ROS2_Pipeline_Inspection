import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import threading
import time
import serial
from serial import SerialException

class SerialPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')

        self.publisher_AccX = self.create_publisher(Float32, 'serial_data/AccX', 10)
        self.publisher_AccY = self.create_publisher(Float32, 'serial_data/AccY', 10)
        self.publisher_AccZ = self.create_publisher(Float32, 'serial_data/AccZ', 10)
        
        self.publisher_AsX = self.create_publisher(Float32, 'serial_data/AsX', 10)
        self.publisher_AsY = self.create_publisher(Float32, 'serial_data/AsY', 10)
        self.publisher_AsZ = self.create_publisher(Float32, 'serial_data/AsZ', 10)
        
        self.publisher_HX = self.create_publisher(Float32, 'serial_data/HX', 10)
        self.publisher_HY = self.create_publisher(Float32, 'serial_data/HY', 10)
        self.publisher_HZ = self.create_publisher(Float32, 'serial_data/HZ', 10)

        self.publisher_AngX = self.create_publisher(Float32, 'serial_data/AngX', 10)
        self.publisher_AngY = self.create_publisher(Float32, 'serial_data/AngY', 10)
        self.publisher_AngZ = self.create_publisher(Float32, 'serial_data/AngZ', 10)

        self.serial_device = DeviceModel("test equipment", "/dev/ttyUSB1", 9600, 0x50, self.update_data)
        self.serial_device.openDevice()
        self.serial_device.startLoopRead()

    def update_data(self, device_data):

        # msg = Float32()
        # msg.data = float(device_data.deviceData)
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)

        msg_AccX = Float32()
        msg_AccX.data = device_data.deviceData['AccX']
        self.publisher_AccX.publish(msg_AccX)

        msg_AccY = Float32()
        msg_AccY.data = device_data.deviceData['AccY']
        self.publisher_AccY.publish(msg_AccY)

        msg_AccZ = Float32()
        msg_AccZ.data = device_data.deviceData['AccZ']
        self.publisher_AccZ.publish(msg_AccZ)

        msg_AsX = Float32()
        msg_AsX.data = device_data.deviceData['AsX']
        self.publisher_AsX.publish(msg_AsX)

        msg_AsY = Float32()
        msg_AsY.data = device_data.deviceData['AsY']
        self.publisher_AsY.publish(msg_AsY)

        msg_AsZ = Float32()
        msg_AsZ.data = device_data.deviceData['AsZ']
        self.publisher_AsZ.publish(msg_AsZ)

        msg_HX = Float32()
        msg_HX.data = device_data.deviceData['HX']
        self.publisher_HX.publish(msg_HX)

        msg_HY = Float32()
        msg_HY.data = device_data.deviceData['HY']
        self.publisher_HY.publish(msg_HY)

        msg_HZ = Float32()
        msg_HZ.data = device_data.deviceData['HZ']
        self.publisher_HZ.publish(msg_HZ)

        self.get_logger().info('Publishing: AccX=%s, AccY=%s, AccZ=%s, AsX=%s, AsY=%s, AsZ=%s, HX=%s, HY=%s, HZ=%s' % (msg_AccX.data, msg_AccY.data, msg_AccZ.data, msg_AsX.data, msg_AsY.data, msg_AsZ.data, msg_HX.data, msg_HY.data, msg_HZ.data))


def main(args=None):
    rclpy.init(args=args)
    serial_publisher = SerialPublisher()
    rclpy.spin(serial_publisher)
    serial_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# 串口配置 Serial Port Configuration
class SerialConfig:
    # serial number
    portName = ''

    # baud
    baud = 9600


# 设备实例 Device instance
class DeviceModel:
    deviceName = "My Device"
    ADDR = 0x50
    deviceData = {}
    isOpen = False
    loop = False
    serialPort = None
    serialConfig = SerialConfig()
    TempBytes = []
    statReg = None


    auchCRCHi = [
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
        0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
        0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
        0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
        0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
        0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
        0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
        0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
        0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
        0x40]

    auchCRCLo = [
        0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
        0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
        0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
        0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
        0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
        0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
        0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
        0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
        0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
        0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
        0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
        0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
        0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
        0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
        0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
        0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
        0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
        0x40]


    def __init__(self, deviceName, portName, baud, ADDR, callback_method):
        print("Initialize the device model")
        self.deviceName = deviceName
        self.serialConfig.portName = portName
        self.serialConfig.baud = baud
        self.ADDR = ADDR
        self.deviceData = {}
        self.callback_method = callback_method

    # 获得CRC校验 Obtain CRC verification
    def get_crc(self, datas, dlen):
        tempH = 0xff  # 高 CRC 字节初始化 High CRC byte initialization
        tempL = 0xff  # 低 CRC 字节初始化 Low CRC byte initialization
        for i in range(0, dlen):
            tempIndex = (tempH ^ datas[i]) & 0xff
            tempH = (tempL ^ self.auchCRCHi[tempIndex]) & 0xff
            tempL = self.auchCRCLo[tempIndex]
        return (tempH << 8) | tempL

    def set(self, key, value):
        self.deviceData[key] = value

    def get(self, key):
        if key in self.deviceData:
            return self.deviceData[key]
        else:
            return None

    def remove(self, key):
        del self.deviceData[key]

    def openDevice(self):
        self.closeDevice()
        try:
            self.serialPort = serial.Serial(self.serialConfig.portName, self.serialConfig.baud, timeout=0.5)
            self.isOpen = True
            print("{} Opened ".format(self.serialConfig.portName))
            t = threading.Thread(target=self.readDataTh, args=("Data-Received-Thread", 10,))
            t.start()
            print("Device turned on successfully")
        except SerialException:
            print("turn on " + self.serialConfig.portName + " fail")


    def readDataTh(self, threadName, delay):
        print("launch " + threadName)
        while True:
            if self.isOpen:
                
                tLen = self.serialPort.inWaiting()
                if tLen > 0:
                    data = self.serialPort.read(tLen)
                    self.onDataReceived(data)
               
            else:
                time.sleep(0.1)
                print("Serial port not open")
                break

    def closeDevice(self):
        if self.serialPort is not None:
            self.serialPort.close()
            print("The port is closed")
        self.isOpen = False
        print("The device is off")

    def onDataReceived(self, data):
        tempdata = bytes.fromhex(data.hex())
        for val in tempdata:
            self.TempBytes.append(val)

            if self.TempBytes[0] != self.ADDR:
                del self.TempBytes[0]
                continue
            if len(self.TempBytes) > 2:
                if not (self.TempBytes[1] == 0x03):
                    del self.TempBytes[0]
                    continue
                tLen = len(self.TempBytes)
                if tLen == self.TempBytes[2] + 5:
                    tempCrc = self.get_crc(self.TempBytes, tLen - 2)
                    if (tempCrc >> 8) == self.TempBytes[tLen - 2] and (tempCrc & 0xff) == self.TempBytes[tLen - 1]:
                        self.processData(self.TempBytes[2])
                    else:
                        del self.TempBytes[0]

    def processData(self, length):
        if length == 30:
            AccX = self.getSignInt16(self.TempBytes[3] << 8 | self.TempBytes[4]) / 32768 * 16
            AccY = self.getSignInt16(self.TempBytes[5] << 8 | self.TempBytes[5]) / 32768 * 16
            AccZ = self.getSignInt16(self.TempBytes[7] << 8 | self.TempBytes[8]) / 32768 * 16
            self.set("AccX", round(AccX, 3))
            self.set("AccY", round(AccY, 3))
            self.set("AccZ", round(AccZ, 3))

            AsX = self.getSignInt16(self.TempBytes[9] << 8 | self.TempBytes[10]) / 32768 * 2000
            AsY = self.getSignInt16(self.TempBytes[11] << 8 | self.TempBytes[12]) / 32768 * 2000
            AsZ = self.getSignInt16(self.TempBytes[13] << 8 | self.TempBytes[14]) / 32768 * 2000
            self.set("AsX", round(AsX, 3))
            self.set("AsY", round(AsY, 3))
            self.set("AsZ", round(AsZ, 3))

            HX = self.getSignInt16(self.TempBytes[15] << 8 | self.TempBytes[16]) * 13 / 1000
            HY = self.getSignInt16(self.TempBytes[17] << 8 | self.TempBytes[18]) * 13 / 1000
            HZ = self.getSignInt16(self.TempBytes[19] << 8 | self.TempBytes[20]) * 13 / 1000
            self.set("HX", round(HX, 3))
            self.set("HY", round(HY, 3))
            self.set("HZ", round(HZ, 3))

            AngX = self.getSignInt32(
                self.TempBytes[23] << 24 | self.TempBytes[24] << 16 | self.TempBytes[21] << 8 | self.TempBytes[
                    22]) / 1000
            AngY = self.getSignInt32(
                self.TempBytes[27] << 24 | self.TempBytes[28] << 16 | self.TempBytes[25] << 8 | self.TempBytes[
                    26]) / 1000
            AngZ = self.getSignInt32(
                self.TempBytes[31] << 24 | self.TempBytes[32] << 16 | self.TempBytes[29] << 8 | self.TempBytes[
                    30]) / 1000
            self.set("AngX", round(AngX, 3))
            self.set("AngY", round(AngY, 3))
            self.set("AngZ", round(AngZ, 3))
            self.callback_method(self)
        else:
            if self.statReg is not None:
                for i in range(int(length / 2)):
                    value = self.getSignInt16(self.TempBytes[2 * i + 3] << 8 | self.TempBytes[2 * i + 4])
                    value = value / 32768
                    self.set(str(self.statReg), round(value, 3))
                    self.statReg += 1
        self.TempBytes.clear()

    @staticmethod
    def getSignInt16(num):
        if num >= pow(2, 15):
            num -= pow(2, 16)
        return num

    @staticmethod
    def getSignInt32(num):
        if num >= pow(2, 31):
            num -= pow(2, 32)
        return num
    def sendData(self, data):
        try:
            self.serialPort.write(data)
        except Exception as ex:
            print(ex)


    def readReg(self, regAddr, regCount):
        self.statReg = regAddr
        self.sendData(self.get_readBytes(self.ADDR, regAddr, regCount))

    def writeReg(self, regAddr, sValue):
        self.unlock()
        time.sleep(0.1)
        self.sendData(self.get_writeBytes(self.ADDR, regAddr, sValue))
        time.sleep(0.1)
        self.save()

    def get_readBytes(self, devid, regAddr, regCount):
        tempBytes = [None] * 8
        tempBytes[0] = devid
        tempBytes[1] = 0x03
        tempBytes[2] = regAddr >> 8
        tempBytes[3] = regAddr & 0xff
        tempBytes[4] = regCount >> 8
        tempBytes[5] = regCount & 0xff
        tempCrc = self.get_crc(tempBytes, len(tempBytes) - 2)
        tempBytes[6] = tempCrc >> 8
        tempBytes[7] = tempCrc & 0xff
        return tempBytes

    def get_writeBytes(self, devid, regAddr, sValue):
        tempBytes = [None] * 8
        tempBytes[0] = devid
        tempBytes[1] = 0x06
        tempBytes[2] = regAddr >> 8
        tempBytes[3] = regAddr & 0xff
        tempBytes[4] = sValue >> 8
        tempBytes[5] = sValue & 0xff
        tempCrc = self.get_crc(tempBytes, len(tempBytes) - 2)
        tempBytes[6] = tempCrc >> 8
        tempBytes[7] = tempCrc & 0xff
        return tempBytes

    def startLoopRead(self):
        self.loop = True
        t = threading.Thread(target=self.loopRead, args=())
        t.start()

    def loopRead(self):
        print("Loop read start")
        while self.loop:
            self.readReg(0x34, 15)
            time.sleep(0.2)
        print("End of loop reading")

    def stopLoopRead(self):
        self.loop = False

    def unlock(self):
        cmd = self.get_writeBytes(self.ADDR, 0x69, 0xb588)
        self.sendData(cmd)

    def save(self):
        cmd = self.get_writeBytes(self.ADDR, 0x00, 0x0000)
        self.sendData(cmd)