import serial
import time

# Ganti '/dev/ttyUSB0' sesuai port ESP32 (cek dengan 'ls /dev/ttyUSB*')
esp32 = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # Tunggu koneksi serial stabil

try:
    # Kirim perintah '1' ke ESP32
    esp32.write(b'1\n')
    print(">> Kirim ke ESP32: 1")

    # Tunggu dan baca balasan dari ESP32
    while True:
        #esp32.write(b'1\n')
        #print(">> Kirim ke ESP32: 1")
        if esp32.in_waiting > 0:
            response = esp32.readline().decode().strip()
            print("<< Balasan dari ESP32:", response)
            
            # Jika ESP32 kirim status 'TERTUTUP' atau 'TERBUKA', bisa ditindak lanjuti
            if response == "[ESP32] Dapat '1' dari Raspberry Pi - mulai buka pintu":
                print("Status: Pintu berhasil terbuka.")
            elif response == "TERTUTUP":
                print("Status: Pintu sudah tertutup.")
                esp32.write(b'1\n')
                print(">> Kirim ke ESP32: 1")
              # keluar dari loop setelah dapat balasan yang diinginkan
        #time.sleep(1)

except KeyboardInterrupt:
    print("Program dihentikan")

finally:
    esp32.close()
