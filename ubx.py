import time
import struct

PREAMBLE1 = 0xb5
PREAMBLE2 = 0x62

# message classes
CLASS_NAV = 0x01
CLASS_RXM = 0x02
CLASS_INF = 0x04
CLASS_ACK = 0x05
CLASS_CFG = 0x06
CLASS_MON = 0x0A
CLASS_AID = 0x0B
CLASS_TIM = 0x0D
CLASS_ESF = 0x10

# ACK messages
MSG_ACK_NACK = 0x00
MSG_ACK_ACK = 0x01

# NAV messages
MSG_NAV_POSECEF   = 0x1
MSG_NAV_POSLLH    = 0x2
MSG_NAV_STATUS    = 0x3
MSG_NAV_DOP       = 0x4
MSG_NAV_SOL       = 0x6
MSG_NAV_POSUTM    = 0x8
MSG_NAV_VELNED    = 0x12
MSG_NAV_VELECEF   = 0x11
MSG_NAV_TIMEGPS   = 0x20
MSG_NAV_TIMEUTC   = 0x21
MSG_NAV_CLOCK     = 0x22
MSG_NAV_SVINFO    = 0x30
MSG_NAV_SVIN    = 0x3b
MSG_NAV_AOPSTATUS = 0x60
MSG_NAV_DGPS      = 0x31
MSG_NAV_DOP       = 0x04
MSG_NAV_EKFSTATUS = 0x40
MSG_NAV_SBAS      = 0x32
MSG_NAV_SOL       = 0x06
MSG_NAV_SAT       = 0x35

# RXM messages
MSG_RXM_RAW    = 0x10
MSG_RXM_SFRB   = 0x11
MSG_RXM_SVSI   = 0x20
MSG_RXM_EPH    = 0x31
MSG_RXM_ALM    = 0x30
MSG_RXM_PMREQ  = 0x41

# AID messages
MSG_AID_ALM    = 0x30
MSG_AID_EPH    = 0x31
MSG_AID_ALPSRV = 0x32
MSG_AID_AOP    = 0x33
MSG_AID_DATA   = 0x10
MSG_AID_ALP    = 0x50
MSG_AID_DATA   = 0x10
MSG_AID_HUI    = 0x02
MSG_AID_INI    = 0x01
MSG_AID_REQ    = 0x00

# CFG messages
MSG_CFG_PRT = 0x00
MSG_CFG_ANT = 0x13
MSG_CFG_DAT = 0x06
MSG_CFG_EKF = 0x12
MSG_CFG_ESFGWT = 0x29
MSG_CFG_CFG = 0x09
MSG_CFG_USB = 0x1b
MSG_CFG_RATE = 0x08
MSG_CFG_SET_RATE = 0x01
MSG_CFG_NAV5 = 0x24
MSG_CFG_FXN = 0x0E
MSG_CFG_INF = 0x02
MSG_CFG_ITFM = 0x39
MSG_CFG_MSG = 0x01
MSG_CFG_NAVX5 = 0x23
MSG_CFG_NMEA = 0x17
MSG_CFG_NVS = 0x22
MSG_CFG_PM2 = 0x3B
MSG_CFG_PM = 0x32
MSG_CFG_RINV = 0x34
MSG_CFG_RST = 0x04
MSG_CFG_RXM = 0x11
MSG_CFG_SBAS = 0x16
MSG_CFG_TMODE = 0x1D
MSG_CFG_TMODE2 = 0x3D
MSG_CFG_TMODE3 = 0x71
MSG_CFG_TPS = 0x31
MSG_CFG_TP = 0x07
MSG_CFG_GNSS = 0x3E

# ESF messages
MSG_ESF_MEAS   = 0x02
MSG_ESF_STATUS = 0x10

# INF messages
MSG_INF_DEBUG  = 0x04
MSG_INF_ERROR  = 0x00
MSG_INF_NOTICE = 0x02
MSG_INF_TEST   = 0x03
MSG_INF_WARNING= 0x01

# MON messages
MSG_MON_SCHD  = 0x01
MSG_MON_HW    = 0x09
MSG_MON_HW2   = 0x0B
MSG_MON_IO    = 0x02
MSG_MON_MSGPP = 0x06
MSG_MON_RXBUF = 0x07
MSG_MON_RXR   = 0x21
MSG_MON_TXBUF = 0x08
MSG_MON_VER   = 0x04

# TIM messages
MSG_TIM_TP   = 0x01
MSG_TIM_TM2  = 0x03
MSG_TIM_SVIN = 0x04
MSG_TIM_VRFY = 0x06

# port IDs
PORT_DDC    =0
PORT_SERIAL1=1
PORT_SERIAL2=2
PORT_USB    =3
PORT_SPI    =4

# dynamic models
DYNAMIC_MODEL_PORTABLE   = 0
DYNAMIC_MODEL_STATIONARY = 2
DYNAMIC_MODEL_PEDESTRIAN = 3
DYNAMIC_MODEL_AUTOMOTIVE = 4
DYNAMIC_MODEL_SEA        = 5
DYNAMIC_MODEL_AIRBORNE1G = 6
DYNAMIC_MODEL_AIRBORNE2G = 7
DYNAMIC_MODEL_AIRBORNE4G = 8

#reset items
RESET_HOT  = 0
RESET_WARM = 1
RESET_COLD = 0xFFFF

RESET_HW            = 0
RESET_SW            = 1
RESET_SW_GPS        = 2
RESET_HW_GRACEFUL   = 4
RESET_GPS_STOP      = 8
RESET_GPS_START     = 9

class UbxParseManager(object):
    def __init__(self):
        self.step = 0
        self.class_id = None
        self.msg_id = None
        self.msg_len = None
        self.payload = bytearray()
        self.origin_buf = bytearray()

        self.status = {}
        self.last_parsed = 0

        self.ck_a = 0
        self.ck_b = 0
    
    def update_crc(self, val):
        self.ck_a = (self.ck_a + val) & 0xFF
        self.ck_b = (self.ck_b + self.ck_a) & 0xFF

    def next(self):
        self.step += 1

    def reset(self):
        self.step = 0
        self.class_id = None
        self.msg_id = None
        self.msg_len = None
        self.payload = bytearray()
        self.ck_a = 0
        self.ck_b = 0
        self.origin_buf = bytearray()

    def is_active(self):
        return time.time() - self.last_parsed < 1000

    def wrap_ubx_crc(self, buf):
        ck_a = 0
        ck_b = 0
        for i in buf[2:]:
            ck_a = (ck_a + i) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF

        return buf + struct.pack("<BB", ck_a, ck_b)

    def enter_svin(self, version=0x00, reserved1=0x00, mode=0x01, lla=0x00, ecefXOrLat=0, ecefXOrLon=0, ecefXOrAlt=0, ecefXOrLatHP=0, ecefYOrLonHP=0, ecefZOrAltHP=0, reserved2=0, fixedPosAcc=0, svinMinDur=200, svinAccLimit=20000, reserved3=0):
        buf = struct.pack("<4B H 2B 2B 3i 3b B I", PREAMBLE1, PREAMBLE2, CLASS_CFG, MSG_CFG_TMODE3, 40, version, reserved1, mode, lla, ecefXOrLat, ecefXOrLon, ecefXOrAlt, ecefXOrLatHP, ecefYOrLonHP, ecefZOrAltHP, reserved2, fixedPosAcc)

        buf += struct.pack('<I', svinMinDur)
        buf += struct.pack('<I', svinAccLimit)
        buf += struct.pack("<q", reserved3)

        return self.wrap_ubx_crc(buf)
        #  print(len(self.wrap_ubx_crc(buf)), struct.unpack('<{0}B'.format(len(self.wrap_ubx_crc(buf))), self.wrap_ubx_crc(buf)))
        print("ublox enter_svin")
    
    def parse(self, class_id, msg_id, payload):
        self.last_parsed = time.time()
        if class_id == CLASS_NAV and msg_id == MSG_NAV_SVIN:
            #  print("MSG_NAV_SVIN")
            res = struct.unpack("<b 3b I I 3i 3b B II B B 2B", payload)

            self.status["MSG_NAV_SVIN"] = {
                    'version': res[0],
                    'resserved1': res[1:4],
                    'iTOW': res[4],
                    'dur': res[5],
                    'meanX': res[6],
                    'meanY': res[7],
                    'meanZ': res[8],
                    'meanXHP': res[9],
                    'meanYHP': res[10],
                    'meanZHP': res[11],
                    'resserved2': res[12],
                    'meanAcc': res[13],
                    'obs': res[14],
                    'valid': res[15],
                    'active': res[16],
                    'resserved3': res[17:19],
                    }

            #  print(self.status["MSG_NAV_SVIN"])

        elif class_id == CLASS_NAV and msg_id == MSG_NAV_CLOCK:
            #  print("MSG_NAV_CLOCK")
            res = struct.unpack("<IiiII", payload)
            self.status["MSG_NAV_CLOCK"] = {
                    "iTOW": res[0],
                    "clkB": res[1],
                    "clkD": res[2],
                    "tAcc": res[3],
                    "fAcc": res[4],
                    }
            #  print(self.status["MSG_NAV_CLOCK"])

        elif class_id == CLASS_NAV and msg_id == MSG_NAV_SAT:
            #  print("MSG_NAV_SAT")
            res = struct.unpack("<IBBBB", payload[0:8])
            new_state = {
                    "iTOW": res[0],
                    "version": res[1],
                    "numSvs": res[2],
                    "reserved1": res[3:5],
                    "svs": []
                    }

            for i in range(new_state['numSvs']):
                res = struct.unpack("<BBBbhhI", payload[8 + i*12: 20 + i*12])
                flags = '{0:032b}'.format(res[6])[::-1]
                new_state['svs'].append({
                    "gnssId": res[0],
                    "svId": res[1],
                    "cno": res[2],
                    "elev": res[3],
                    "azim": res[4],
                    "prRes": res[5],
                    "qualityInd": int(flags[0:3], 2),
                    "svUsed": flags[3] == '1',
                    "health": int(flags[4:6], 2),
                    "diffCorr": flags[6] == '1',
                    "smoothed": flags[7] == '1',
                    "orbitSource": int(flags[8: 11], 2),
                    "ephAvail": flags[11] == '1',
                    "almAvail": flags[12] == '1',
                    "anoAvail": flags[13] == '1',
                    "aopAvail": flags[14] == '1',
                    "sbasCorrUsed": flags[16] == '1',
                    "rtcmCorrUsed": flags[17] == '1',
                    "slasCorrUsed": flags[18] == '1',
                    "prCorrUsed": flags[20] == '1',
                    "crCorrUsed": flags[21] == '1',
                    "doCorrUsed": flags[22] == '1',
                    })

            self.status["MSG_NAV_SAT"] = new_state
            #  print(self.status["MSG_NAV_SAT"])
