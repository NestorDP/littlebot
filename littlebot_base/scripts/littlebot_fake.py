#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 26 09:09:20 2018

@author: nestor
"""

import base64
import serial
import sys

import todolist_pb2 as TodoList

serial_port = sys.argv[1]

ser = serial.Serial(serial_port, baudrate=115200)  # open serial port

start_character = b'<'
end_character = b'>'

# Create a new TodoList
my_list = TodoList.TodoList()
my_list.owner_id = 1234

my_list.owner_name = "Tim"

first_item = my_list.todos.add()
first_item.state = TodoList.TaskState.Value("TASK_DONE")
first_item.task = "Test ProtoBuf for Python"
first_item.due_date = "31.10.2019"

# Serialize the TodoList and send it over the serial port
encoded_data = start_character + my_list.SerializeToString() + end_character
ser.write(encoded_data)
print(f"Encoded data sent: {encoded_data}\n")

ser.flush()

my_list2 = TodoList.TodoList()

# Read the encoded data from the serial port
data = ser.read()
if data == start_character:
    encoded_data_received = ser.read_until(end_character)

# Debugging print
print(f"Encoded data received: {encoded_data_received}")

# Parse the decoded data into a TodoList
try:
    my_list2.ParseFromString(encoded_data_received[:-1])
    print(my_list2)
except Exception as e:
    print(f"Failed to parse the message: {e}")

ser.close()