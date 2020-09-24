# -*- coding: utf-8 -*-


import serial
import os
import time


arduino_voltage = serial.Serial('/dev/ttyACM1', 9600)
time.sleep(3)
# arduino_voltage.write('1')


def send_voltage():
    arduino_voltage.write('1')
    time.sleep(2)
    voltage = arduino_voltage.readline()
    print (str(voltage))
    voltage_file = open("voltage.txt", "w")
    # voltage_file.seek(0)
    voltage_file.write(str(voltage))
    voltage_file.close()
#
if __name__ == "__main__":
    send_voltage()