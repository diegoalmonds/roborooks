#!/usr/bin/env python3

import serial
import time

PORT = "/dev/ttyACM1"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

time.sleep(1)

def magnet_on():
    cmd = "PIN 9 ON\n"
    ser.write(cmd.encode())
    print("MAGNET ON")
    print(ser.readline().decode().strip())
    print(ser.readline().decode().strip())
    print(ser.readline().decode().strip())

def magnet_off():
    cmd = "PIN 9 OFF\n"
    ser.write(cmd.encode())
    print("MAGNET OFF")
    print(ser.readline().decode().strip())
    print(ser.readline().decode().strip())
    print(ser.readline().decode().strip())

while True:
    command = input("Enter 'on', 'off', or 'q': ").strip()
    if command == 'on':
        magnet_on()
    elif command == "off":
        magnet_off()
    else:
        break

ser.close()