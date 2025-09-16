import serial
import time

# === SETTING ===
PORT = '/dev/ttyUSB0'   # Ganti jika bukan ttyUSB0 (lihat dengan `dmesg | grep tty`)
BAUDRATE = 115200       # Sama seperti Serial.begin(115200) di ESP32

# === INISIALISASI ===
try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=2)
    time.sleep(2)  # Waktu tunggu ESP32 siap
    print("[RPi] Serial siap")
except Exception as e:
    print("[RPi] Gagal buka port:", e)
    exit()

# === LOOP UTAMA ===
while True:
    print("\n=== MENU ===")
    print("1. Kirim BUKA (1)")
    print("2. Kirim TUTUP (0)")
    print("3. Keluar")
    pilihan = input("Pilih opsi: ")

    if pilihan == "1":
        ser.write(b"1\n")
        print("[RPi] Dikirim: 1 (BUKA)")
    elif pilihan == "2":
        ser.write(b"0\n")
        print("[RPi] Dikirim: 0 (TUTUP)")
    elif pilihan == "3":
        print("[RPi] Keluar program.")
        break
    else:
        print("Input tidak dikenal.")
        continue

    # Tunggu respons dari ESP32
    waktu_tunggu = time.time() + 5  # timeout 5 detik
    while time.time() < waktu_tunggu:
        if ser.in_waiting:
            line = ser.readline().decode().strip()
            if line.startswith("RESP:"):
                status = line.replace("RESP:", "")
                print(f"[RPi] Status dari ESP32: {status}")
                break
        time.sleep(0.1)

ser.close()
