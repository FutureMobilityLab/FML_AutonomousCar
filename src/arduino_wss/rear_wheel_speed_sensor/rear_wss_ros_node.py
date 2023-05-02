#!/usr/bin/env python3
import serial

if __name__ == '__main__':
    # Setup serial connection with 9600 baud rate and a timeout of 1 sec.
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    # Reset the input buffer to clear incomplete data.
    ser.reset_input_buffer()

    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            print(line)
