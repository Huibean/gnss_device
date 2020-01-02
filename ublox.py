import struct
from datetime import datetime
import time, os
import queue

import numpy as np
from .ubx import UbxParseManager, PREAMBLE1, PREAMBLE2
from .rtcm3 import RTCM3ParseManager, RTCM3_HEADER

UBX = 0x01
RTCM3 = 0x02

class UBlox:
    def __init__(self, port, baudrate=115200, timeout=0, proxy=None):
        self.serial_device = port
        self.baudrate = baudrate
        if self.serial_device.startswith("tcp:"):
            import socket
            a = self.serial_device.split(':')
            destination_addr = (a[1], int(a[2]))
            self.dev = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.dev.connect(destination_addr)
            self.dev.setblocking(1)
            self.dev.setsockopt(socket.SOL_TCP, socket.TCP_NODELAY, 1)            
            self.use_sendrecv = True
        elif os.path.isfile(self.serial_device):
            self.read_only = True
            self.dev = open(self.serial_device, mode='rb')
        else:
            import serial
            self.dev = serial.Serial(self.serial_device, baudrate=self.baudrate,dsrdtr=False, rtscts=False, xonxoff=False, timeout=timeout)

        self.proxy = proxy
        self.ubxParseManager = UbxParseManager()
        self.rtcm3ParseManager = RTCM3ParseManager()
        self.rtcm_queue = queue.Queue()

    def read(self, n):
        return self.dev.read(n)

    def write(self, buf):
        return self.dev.write(buf)

    @property
    def gnss_count(self):
        count = 0
        if 'MSG_NAV_SAT' in self.status:
            for satellite in self.status['MSG_NAV_SAT']['svs']:
                if satellite['svUsed']:
                    count += 1
            
        return count

    @property
    def is_survey_in_success(self):
        flag = False
        if 'MSG_NAV_SVIN' in self.status:
            flag = self.status['valid']

        return flag

    @property
    def is_survey_in_processing(self):
        flag = False
        if 'MSG_NAV_SVIN' in self.status:
            flag = self.status['active']

        return flag

    @property
    def survey_in_acc(self):
        acc = 1000000
        if 'MSG_NAV_SVIN' in self.status:
            acc = int(self.status['meanAcc'] * 10)

        return acc

    @property
    def status(self):
        return self.ubxParseManager.status

    def loop(self):
        ubxParseManager = self.ubxParseManager
        rtcm3ParseManager = self.rtcm3ParseManager
        rtcm_queue = self.rtcm_queue
        current_type = -1

        while True:
            data = self.read(1)
            if not data:
                continue

            if ubxParseManager.step == 0 and rtcm3ParseManager.size == 0:
                header = int.from_bytes(data, 'big')
                if header == PREAMBLE1:
                    current_type = UBX
                elif header == RTCM3_HEADER:
                    current_type = RTCM3

            if current_type == RTCM3:
                rtcm3ParseManager.add(data)
                rtcm3ParseManager.origin_buf += data
                if rtcm3ParseManager.is_full():
                    rtcm_queue.put(rtcm3ParseManager.buf)

                    if self.proxy:
                        self.proxy.send(rtcm3ParseManager.origin_buf)

                    rtcm3ParseManager.reset()
                    current_type = -1

            if current_type == UBX:
                val = int.from_bytes(data, 'big')
                ubxParseManager.origin_buf += data
                if ubxParseManager.step == 0:
                    if val == PREAMBLE1:
                        ubxParseManager.next()

                elif ubxParseManager.step == 1:
                    if val == PREAMBLE2:
                        ubxParseManager.next()
                    else:
                        ubxParseManager.reset()

                elif ubxParseManager.step == 2:
                    ubxParseManager.next()
                    ubxParseManager.class_id = val

                    ubxParseManager.update_crc(val)

                elif ubxParseManager.step == 3:
                    ubxParseManager.next()
                    ubxParseManager.msg_id = val
                    ubxParseManager.update_crc(val)

                elif ubxParseManager.step == 4:
                    ubxParseManager.next()
                    ubxParseManager.update_crc(val)
                    ubxParseManager.msg_len = val

                elif ubxParseManager.step == 5:
                    ubxParseManager.next()
                    ubxParseManager.update_crc(val)
                    ubxParseManager.msg_len += val<<8

                elif ubxParseManager.step == 6:
                    ubxParseManager.payload += data
                    ubxParseManager.update_crc(val)
                    if len(ubxParseManager.payload) == ubxParseManager.msg_len:
                        ubxParseManager.next()

                elif ubxParseManager.step == 7:
                    ubxParseManager.next()
                    if ubxParseManager.ck_a != val:
                        print(f'ublox bad cka{ubxParseManager.msg_id}, {ubxParseManager.ck_a}, {val}')
                        ubxParseManager.reset()

                elif ubxParseManager.step == 8:
                    if ubxParseManager.ck_b != val:
                        print(f'ublox bad cka{ubxParseManager.msg_id}, {ubxParseManager.ck_b}, {val}')
                    else:
                        ubxParseManager.parse(ubxParseManager.class_id, ubxParseManager.msg_id, ubxParseManager.payload)
                        if self.proxy:
                            self.proxy.send(ubxParseManager.origin_buf)

                    ubxParseManager.reset()
