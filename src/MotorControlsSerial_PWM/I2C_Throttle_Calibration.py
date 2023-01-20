#!/usr/bin/env python

import time
import lgpio

ESC_Address = 0x08
h_ESC = lgpio.i2c_open(1, ESC_Address)

#	ESC Registers and their Address
# READ ADDRESSES
ESC_VOLTAGE = 0x0
ESC_RIPPLE = 0x1
ESC_CURRENT = 0x2
ESC_THROTTLE = 0x3
ESC_POWER = 0x4
ESC_SPEED = 0x5
ESC_TEMP = 0x6
ESC_BEC_VOLT = 0x7
ESC_BEC_CURRENT = 0x8
ESC_RAW_NTC = 0x9
ESC_RAW_LINEAR = 0x10
ESC_LINK_LIVE = 0x25
ESC_FAIL_SAFE = 0x26
ESC_E_STOP = 0x27
ESC_PACKET_IN = 0x28
ESC_PACKET_OUT = 0x29
ESC_CHECK_BAD = 0x30
ESC_PACKET_BAD = 0x31
#WRITE ADDRESSES
ESC_CMD_THROTTLE = 0x80		#0-65535
ESC_CMD_FAIL_SAFE = 0x129		#0-100
ESC_CMD_E_STOP = 0x130			#0/1
ESC_CMD_PACKET_IN = 0x131		#SETS TO 0
ESC_CMD_CHECK_BAD = 0x132		#SETS TO 0
ESC_CMD_PACKET_BAD = 0x133		#SETS TO 0

def throttleCalibrationState(count):
        if count == 0:
                setp = 65535

        if count == 1:
                setp = 0

        if count == 2:
                setp = 32767

        return setp

count = 0
while True:
        try:
                while True:
                        print('Throttle State: ', count,end="\r")
                        ThrottleSetpoint = throttleCalibrationState(count)
                        lgpio.i2c_write_word_data(h_ESC,ESC_CMD_THROTTLE,ThrottleSetpoint)
        except KeyboardInterrupt:
                count += 1        

        



