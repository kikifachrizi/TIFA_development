import pandas as pd

def process_csv(input_file, output_file):
    # Membaca file CSV
    data = pd.read_csv(input_file)
    
    # Membatasi angka di belakang koma hanya satu digit untuk x dan y
    data['x'] = data['x'] * 100#.round()
    data['y'] = data['y'] * 100#.round()

    # Menghapus duplikasi berdasarkan kolom 'x' dan 'y'
    data = data.drop_duplicates(subset=['x', 'y'])
    
    # Menyimpan hasil ke file CSV baru
    data.to_csv(output_file, index=False)
    print(f"File telah disimpan ke {output_file}")

if __name__ == "__main__":
    input_file = '~/catkin_ws/src/pid_proccess/scripts/trajectory.csv'  # Ganti dengan path file input kamu
    output_file = '~/catkin_ws/src/pid_proccess/scripts/trajectorycoba.csv'  # Ganti dengan path file output yang diinginkan
    process_csv(input_file, output_file)
