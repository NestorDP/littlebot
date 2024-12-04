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

class LittlebotFake:
    START_CHARACTER = b'<'
    END_CHARACTER = b'>'

    def __init__(self, serial_port):
        self.ser = self.serial_configuration(serial_port)
        self.littlebot_msg = LittlebotProtocol.LittlebotProtocol()
        self.radius = 0.037
        self.delta_time = 0.1

    def serial_configuration(self, serial_port):
        return serial.Serial(serial_port, baudrate=115200)

    def linear_velocity_to_angular_position(self, linear_velocity, time):
        angular_velocity = linear_velocity / self.radius
        angular_position = angular_velocity * time
        return angular_position

    def send_status(self):        
        self.littlebot_msg.left_status_vel = self.left_command_vel
        self.littlebot_msg.left_status_pos = self.linear_velocity_to_angular_position(self.left_command_vel, self.delta_time)

        self.littlebot_msg.right_status_vel = self.right_command_vel
        self.littlebot_msg.right_status_pos = self.linear_velocity_to_angular_position(self.right_command_vel, self.delta_time)        

        encoded_data = self.START_CHARACTER + self.self.littlebot_msg.SerializeToString() + self.END_CHARACTER
        self.ser.write(encoded_data)

        threading.Timer(self.delta_time, self.send_status).start()

    def receive_command_callback(self, encoded_data_received):
        try:
            self.littlebot_msg.ParseFromString(encoded_data_received[:-1])
        except Exception as e:
            print(f"Failed to parse the message: {e}")

        self.left_command_vel = self.littlebot_msg.left_command_vel
        self.right_command_vel = self.littlebot_msg.right_command_vel


    def read_from_serial(self):
        while True:
            data = self.ser.read()
            if data == self.START_CHARACTER:
                encoded_data_received = self.ser.read_until(self.END_CHARACTER)
                self.receive_command_callback(encoded_data_received)

    def start(self):
        # Send a message periodicaly
        self.send_status()

        # Start a thread to read from the serial port and call the callback function
        read_thread = threading.Thread(target=self.read_from_serial)
        read_thread.daemon = True
        read_thread.start()

        # Keep the main thread alive
        try:
            while True:
                pass
        except KeyboardInterrupt:
            self.ser.close()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python littlebot_fake.py <serial_port>")
        sys.exit(1)

    serial_port = sys.argv[1]
    
    littlebot_fake = LittlebotFake(serial_port)
    littlebot_fake.start()
