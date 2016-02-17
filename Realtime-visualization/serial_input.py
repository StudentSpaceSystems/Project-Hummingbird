import serial
import numpy as np
import socket
import sys.stdout
import sys.stderr


class SerialReader:
    """Reader Object\n
    Reads from serial port (COM# on Windows, or mount-point for USB on Linux/Mac (possibly /dev/ttys#).\n\n

    __init__(self, new_path) -- instantiates Serial object from PySerial\n
    read_from_serial(COM = None) -- reads from serial object, returns NoneType if unable to read"""

    def __init__(self, new_path, baud=115200, timeout=0):
        """
        Instantiates SerialReader object
        :rtype: returnContainer
        :param: new_path: basestring
        :param: baud: int
        :param: timeout: None, int, float
        """
        self.COM_PATH = new_path
        self.BAUD = baud
        self.SERIAL_OBJECT = None
        self.TIMEOUT = timeout
        self.SERIAL_OBJECT = serial.Serial(self.COM_PATH, self.BAUD, timeout=0)

    def read_from_serial(self):
        """
        Reads line from serial-port at designated baudrate
        :rtype: ReturnContainer
        """
        try:
            return self.SERIAL_OBJECT.readline()
        except serial.SerialTimeoutException:
            return ReturnContainer(1, "Unable to read data. Refer to serial.Serial.SerialTimeoutException")


class SerialServer:
    def __init__(self, ip = "127.0.0.1", port = 6246, serial_address = "COM3"):
        self.UDP_IP = ip
        self.UDP_PORT = port
        self.serial_address = serial_address
        print("Writing data from %s to %s:%s"%(self.serial_address, self.UDP_IP, self.UDP_PORT), file=sys.stdout)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto("err", (self.UDP_IP, self.UDP_PORT))