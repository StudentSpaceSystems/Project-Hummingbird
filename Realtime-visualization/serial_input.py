import serial
import struct


class SerialReader:
    """SerialReader Class\n
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
        self.SERIAL_OBJECT = serial.Serial(self.COM_PATH, self.BAUD, timeout=0, write_timeout=0)

    def bytes_to_float(self, byte_string, num_floats=1):
        return struct.unpack('%sf'%num_floats, byte_string)

    def byte_to_int(self, byte_string, num_ints=1):
        return struct.unpack('%si'%num_ints, byte_string)

    def byte_to_byte(self, byte_string, num_bytes=1):
        return struct.unpack('%sb'%num_bytes, byte_string)


def read_from_serial(lock, serial_object, target_location=[], unpack_type="float"):
    internal_buffer = {}
    while daemon_live:
        if serial_object.SERIAL_OBJECT.read() == b'\x01':
            pass
