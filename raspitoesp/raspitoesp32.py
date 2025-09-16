import serial
import time

# Ganti port sesuai output dari `ls /dev/ttyUSB*`
ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
time.sleep(2)  # tunggu ESP32 siap

try:
    while True:
        # Kirim perintah ke ESP32
        cmd = input("Ketik '1' untuk buka, '0' untuk tutup: ").strip()
        if cmd in ["1", "0"]:
            ser.write((cmd + "\n").encode())
            print("Perintah terkirim:", cmd)

            # Tunggu balasan dari ESP32
            response = ser.readline().decode().strip()
            if response:
                print("Balasan ESP32:", response)
        else:
            print("Perintah tidak valid.")

except KeyboardInterrupt:
    print("\nProgram dihentikan")
    ser.close()
