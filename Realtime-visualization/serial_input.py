import serial
import struct
import sys.stderr

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
        self.SERIAL_OBJECT = serial.Serial(self.COM_PATH, self.BAUD, timeout=0.005, write_timeout=0)

    def bytes_to_float(self, byte_string, num_floats=1):
        return struct.unpack('%sf'%num_floats, byte_string)

    def byte_to_int(self, byte_string, num_ints=1):
        return struct.unpack('%si'%num_ints, byte_string)

    def byte_to_byte(self, byte_string, num_bytes=1):
        return struct.unpack('%sb'%num_bytes, byte_string)

def read_from_serial(lock, serial_object, target_location=[], unpack_type="float"):

    def parse_header():
        byte_string = current_frame[1] + current_frame[2]
        (field_A, field_B) = struct.unpack('2B', byte_string)
        flags = field_A & 240 >> 4
        data_size = field_A & 7 >> 0
        is_evil = field_A & 8 >> 3
        number_streams = field_B & 15
        channel_id = field_B & 240 >> 4
        packet_id = flags & 1
        return number_streams, channel_id, packet_id, flags, data_size, is_evil

    def summarize_packet():
        print("Channel number:\t\t\t%s\n"
              "Number of streams:\t\t%s\n"
              "Data length (bytes):\t\t%s\n"
              "Flag value:\t\t\t%s %s %s\n"
              "Evil bit:\t\t\t%s\n"
              "Data points:\t\t"%(channel_id, num_streams, data_size, flags & 4 >> 2, flags & 2 >> 1, flags & 1, evil_bit), end="")
        for data in data_slice:
            print(data, end="")
        print(end="\n")

    def check_validity():
        pass

    internal_buffer = {}
    packet_index = -1

    data_size = -1
    evil_bit = -1
    flags = -1
    channel_id = -1
    num_streams = -1
    packet_size = -1
    current_frame = []
    data_slice = []

    capture = False
    analysis = False

    while DAEMON_LIVE:
        new_byte = serial_object.SERIAL_OBJECT.read()
        if capture and analysis:
            current_frame.append(new_byte)
            if len(current_frame) == packet_size:
                capture = False
                analysis = False
        elif capture and not analysis:
            current_frame.append(new_byte)
            if len(current_frame) == 3:
                check_validity()
                analysis = True
        elif not capture:
            if new_byte == b'\x01':
                current_frame.append(new_byte)
                capture = True
        else:
            print("Unhandled I/O state!", file=sys.stderr)