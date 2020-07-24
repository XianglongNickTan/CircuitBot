import serial
import time

arduino = serial.Serial('/dev/ttyUSB0', 9600)
time.sleep(2)

print('1 on, 0 off')

while 1:
    str = input()

    if str == '1':
        arduino.write('1'.encode())

    elif str == '0':
        arduino.write('0'.encode())

# ser.close()
