#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 26 09:09:20 2018

@author: nestor
"""

import base64
import serial
import sys

import littlebot_protocol_pb2 as LittlebotProtocol

serial_port = sys.argv[1]

ser = serial.Serial(serial_port, baudrate=115200)  # open serial port

start_character = b'<'
end_character = b'>'

# Create a new TodoList
littlebot_msg_protocol = LittlebotProtocol.LittlebotProtocol()

left_motor = littlebot_msg_protocol.motor_interface.add()
left_motor.side = LittlebotProtocol.MotorSide.Value("LEFT")
left_motor.status_velocity = 10
left_motor.status_position = 20
left_motor.command_velocity = 30

right_motor = littlebot_msg_protocol.motor_interface.add()
right_motor.side = LittlebotProtocol.MotorSide.Value("RIGHT")
right_motor.status_velocity = 40
right_motor.status_position = 50
right_motor.command_velocity = 60

print(littlebot_msg_protocol)

# Serialize the TodoList and send it over the serial port
encoded_data = start_character + littlebot_msg_protocol.SerializeToString() + end_character
ser.write(encoded_data)
print(f"Encoded data sent: {encoded_data}\n")

ser.flush()

littlebot_msg_protocol2 = LittlebotProtocol.LittlebotProtocol()

# Read the encoded data from the serial port
data = ser.read()
if data == start_character:
    encoded_data_received = ser.read_until(end_character)

# Debugging print
print(f"Encoded data received: {encoded_data_received}")

# Parse the decoded data into a TodoList
try:
    littlebot_msg_protocol2.ParseFromString(encoded_data_received[:-1])
    print(littlebot_msg_protocol2)
except Exception as e:
    print(f"Failed to parse the message: {e}")


if left_motor.side == False:
    print("Right motor side is LEFT")

ser.close()