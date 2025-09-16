import serial
import time

ser = serial.Serial('/dev/ttyUSB2', 115200, timeout=1)
time.sleep(2)

ser.write(b'1\n')
print("Dikirim: 1")

while True:
    if ser.in_waiting:
        response = ser.readline().decode().strip()
        print( response)
        if response == "TERBUKA":
            break
