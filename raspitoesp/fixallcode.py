import serial
import time

# Ganti '/dev/ttyUSB0' sesuai port serial ESP32
ESP32_PORT = '/dev/ttyUSB2'
BAUDRATE = 115200

try:
    ser = serial.Serial(ESP32_PORT, BAUDRATE, timeout=1)
    print(f"[Raspberry Pi] Terhubung ke ESP32 di {ESP32_PORT}")
except:
    print("[Raspberry Pi] Gagal membuka koneksi serial.")
    exit()

def kirim_perintah(perintah):
    ser.write((perintah + '\n').encode())
    print(f"[Raspberry Pi] Dikirim ke ESP32: {perintah}")

def baca_dari_esp32():
    try:
        line = ser.readline().decode().strip()
        if line:
            print(f"[ESP32] {line}")
    except:
        pass

def menu():
    print("\n=== MENU ===")
    print("1. Buka Pintu")
    print("2. Keluar")
    print("============")

def loop():
    while True:
        baca_dari_esp32()
        time.sleep(0.1)

        if ser.in_waiting == 0:
            menu()
            pilihan = input("Pilih opsi: ")
            if pilihan == "1":
                kirim_perintah("1")
            elif pilihan == "2":
                print("Keluar.")
                break
            else:
                print("Pilihan tidak dikenal.")
            time.sleep(0.5)
        baca_dari_esp32()
        time.sleep(0.1)

if __name__ == "__main__":
    try:
        loop()
    except KeyboardInterrupt:
        print("\nProgram dihentikan oleh pengguna.")
    finally:
        ser.close()
