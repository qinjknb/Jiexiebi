
class SerialException(Exception):
    pass

class Serial:
    def __init__(self, port=None, baudrate=9600, timeout=None):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.is_open = True
        self.written = []
    def write(self, data):
        self.written.append(bytes(data))
    def read(self, n=1):
        return b""
    @property
    def in_waiting(self):
        return 0
    def close(self):
        self.is_open = False
