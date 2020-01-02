# rtcm3 protocol
RTCM3_HEADER = 0xd3

class RTCM3ParseManager(object):

    def __init__(self):
        self.buf = bytearray()
        self.size = 0
        self.len = 0
        self.origin_buf = bytearray()

    def add(self, data):
        self.buf += data
        self.size += 1

        if self.size == 3:
            packet_len = self.buf[1] & 0x03
            packet_len = packet_len << 8
            packet_len = packet_len | self.buf[2]
            self.len = packet_len + 6

    def is_full(self):
        return self.size > 1023 or (self.len != 0 and (self.size == self.len))

    def reset(self):
        self.size = 0
        self.len = 0
        self.buf = bytearray()
        self.origin_buf = bytearray()
