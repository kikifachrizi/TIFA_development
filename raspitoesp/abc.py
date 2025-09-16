import serial
import time

# Ganti sesuai port ESP32 kamu (misal: /dev/ttyUSB0 atau /dev/ttyS0)
esp32 = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # Tunggu koneksi serial stabil

def kirim_perintah(perintah):
    esp32.write((perintah + '\n').encode())
    print(f">> Kirim ke ESP32: {perintah}")

print("=== MODE KONTROL RASPBERRY PI ===")
print("1 = Buka pintu otomatis")
print("2 = Jalankan motor ESP32")
print("3 = Jalankan motor Arduino")
print("4 = Hanya aktifkan sensor (ESP & Arduino)")
print("0 = Matikan mode sensor-saja")
print("Ctrl+C untuk keluar\n")

buffer_baterai = []

try:
    while True:
        perintah = input("Masukkan perintah (1/2/3/4/0): ").strip()
        if perintah not in ["0", "1", "2", "3", "4"]:
            print("Perintah tidak dikenali.")
            continue

        kirim_perintah(perintah)

        while True:
            if esp32.in_waiting > 0:
                response = esp32.readline().decode(errors="ignore").strip()

                if not response:
                    continue

                print("<<", response)

                # Tangkap data baterai
                if "===== MONITORING TEGANGAN" in response:
                    buffer_baterai = [response]
                elif buffer_baterai:
                    buffer_baterai.append(response)
                    if "====" in response and len(buffer_baterai) >= 6:
                        print("\n--- STATUS BATERAI DARI ESP32 ---")
                        for line in buffer_baterai:
                            print(line)
                        print("----------------------------------\n")
                        buffer_baterai = []

                # Deteksi status pintu
                if response == "[ESP32] Dapat '1' dari Raspberry Pi - mulai buka pintu":
                    print("Status: Mulai buka pintu.")
                elif response == "TERBUKA":
                    print("Status: Pintu sudah terbuka.")
                elif response == "TERTUTUP":
                    print("Status: Pintu tertutup.")
                    break
                elif response == "MOTOR_ESP32_JALAN":
                    print("Status: Motor ESP32 sedang berjalan.")
                    break
                elif response == "MOTOR_ARDUINO_JALAN":
                    print("Status: Motor Arduino sedang berjalan.")
                    break
                elif response == "MODE_SENSOR_SAJA_AKTIF":
                    print("Status: Mode sensor-saja aktif.")
                    break
                elif response == "MODE_SENSOR_SAJA_NONAKTIF":
                    print("Status: Mode sensor-saja dimatikan.")
                    break

except KeyboardInterrupt:
    print("\nProgram dihentikan oleh pengguna.")

finally:
    esp32.close()

