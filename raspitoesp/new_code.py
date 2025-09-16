import serial
import time

# Ganti '/dev/ttyUSB0' sesuai port ESP32 (cek dengan 'ls /dev/ttyUSB*')
esp32 = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # Tunggu koneksi serial stabil

perintah = "0"


#while True:
#    if esp32.in_waiting > 0:
#        response = esp32.readline().decode().strip()
#        print("<< Balasan dari ESP32:", response)

try:
    while True:
        perintah = str(input("Masukkan perintah: "))
        print(esp32.write((str(perintah) + '\n').encode()))
        print(">> Kirim ke ESP32: ", perintah)
        while True:
            if esp32.in_waiting > 0:
                response = esp32.readline().decode().strip()
                print("<< Balasan dari ESP32:", response)

                break

                # # Jika ESP32 kirim status 'TERTUTUP' atau 'TERBUKA', bisa ditindak lanjuti
                # if response == "[ESP32] Dapat '1' dari Raspberry Pi - mulai buka pintu":
                #     print("Status: Pintu berhasil terbuka.")
                # elif response == "TERTUTUP":
                #     print("Status: Pintu sudah tertutup.")
                #     esp32.write(b'1\n')
                #     print(">> Kirim ke ESP32: 1")
        #time.sleep(1)

except KeyboardInterrupt:
    print("Program dihentikan")

finally:
    esp32.close()
