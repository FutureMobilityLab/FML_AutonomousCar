# #!/usr/bin/env python

# Take Command Line Argument 1,2,3 corresponding to calibration stage. 
#       1. Start script at stage 1 before powering on the Castle ESC.
#       2. Power on ESC and wait until 4 beep sequence sounds, confirming full throttle setpoint.
#       3. Run script at stage 2 and wait for additional 4 beeps, confirming full reverse setpoint.
#       4. Run script at stage 3 and wait for final 4 beeps, confirming idle setpoint.

from board import SCL, SDA
import busio
import sys
from adafruit_pca9685 import PCA9685

def ThrottleCalibrationCommand(cmd):
    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50
    pca.channels[1].duty_cycle = cmd
    pca.deinit()

def throttleCalibrationState(throttleStep):
        if throttleStep == 1:
                setp = int(0.1*65535)
                return setp
        if throttleStep == 2:
                setp = int(0.05*65535)
                return setp
        if throttleStep == 3:
                setp = int(0.075*65535)
                return setp
        else:
                sys.exit("Invalid Calibration State")

throttleStep = int(sys.argv[1])
print('Calibration Stage: ',throttleStep)
throttleSetpoint = throttleCalibrationState(throttleStep)
print('Throttle Command: ', throttleSetpoint)
ThrottleCalibrationCommand(throttleSetpoint)