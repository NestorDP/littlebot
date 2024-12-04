#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 26 09:09:20 2018

@author: nestor
"""

import serial
import sys
import threading

import littlebot_protocol_pb2 as LittlebotProtocol

# Define global start and end characters
START_CHARACTER = b'<'
END_CHARACTER = b'>'

def serial_configuration(serial_port):
    serial_port = sys.argv[1]

    # Open serial port
    return serial.Serial(serial_port, baudrate=115200)  # open serial port

def send_message(ser, msg):
    left_motor = msg.motor_interface.add(side=LittlebotProtocol.MotorSide.Value("LEFT"))
    left_motor.status_velocity = 10
    left_motor.status_position = 20

    encoded_data = START_CHARACTER + msg.SerializeToString() + END_CHARACTER
    ser.write(encoded_data)

    threading.Timer(1, send_message, args=(ser, msg)).start()

def receive_message_callback(encoded_data_received):
    print(f"Encoded data received: {encoded_data_received}")


def read_from_serial(ser, callback):
    while True:
        data = ser.read()
        if data == START_CHARACTER:
            encoded_data_received = ser.read_until(END_CHARACTER)
            callback(encoded_data_received)

def main():
    if len(sys.argv) < 2:
        print("Usage: python littlebot_fake.py <serial_port>")
        sys.exit(1)

    serial_port = sys.argv[1]
    ser = serial_configuration(serial_port)

    littlebot_msg_protocol = LittlebotProtocol.LittlebotProtocol()

    # Send a message
    send_message(ser, littlebot_msg_protocol)

    # Start a thread to read from the serial port and call the callback function
    read_thread = threading.Thread(target=read_from_serial, args=(ser, receive_message_callback))
    read_thread.daemon = True
    read_thread.start()

    # Keep the main thread alive
    try:
        while True:
            pass
    except KeyboardInterrupt:
        ser.close()

if __name__ == "__main__":
    main()
