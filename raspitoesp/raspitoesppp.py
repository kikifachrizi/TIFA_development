import serial
import time

ser = serial.Serial('/dev/ttyUSB2', 115200, timeout=1)  # Sesuaikan port
time.sleep(2)

# Kirim perintah buka pintu
ser.write(b'1\n')
print("Dikirim: 1")

# Tunggu respons dari ESP32
while True:
    if ser.in_waiting:
        response = ser.readline().decode().strip()
        print("Diterima dari ESP32:", response)
        if response == "TERBUKA":
            break
