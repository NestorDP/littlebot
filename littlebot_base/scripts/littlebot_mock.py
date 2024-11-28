#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 26 09:09:20 2018

@author: nestor
"""

import serial
import sys

serial_port = sys.argv[1]

ser = serial.Serial(serial_port, baudrate=115200)  # open serial port

ser.write(str("hello\n").encode())         # write a string
ser.flush()
while True:
    # Read one line from the serial port
    if ser.in_waiting > 0:
        data = ser.readline().decode('utf-8').rstrip()
        print(f"Received: {data}")
        if data == "exit":
            break

ser.close()