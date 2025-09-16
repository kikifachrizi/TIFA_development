import pandas as pd

def process_csv_with_averaging(input_file, output_file):
    data = pd.read_csv(input_file)

    if 'x' not in data.columns or 'y' not in data.columns:
        raise ValueError("Kolom 'x' atau 'y' tidak ditemukan dalam file CSV")

    data['x'] = data['x']
    data['y'] = data['y'] * -1
    data['th'] = data['th'] * -1

    averages = []
    for i in range(0, len(data), 20):
        sample = data.iloc[i:i+20]
        avg_x = sample['x'].mean().round(1)
        avg_y = sample['y'].mean().round(1)
        avg_th = sample['th'].mean().round(1)
        averages.append({'x': avg_x, 'y': avg_y, 'th': avg_th})

    averages_df = pd.DataFrame(averages)

    averages_df.to_csv(output_file, index=False)
    print(f"File dengan rata-rata telah disimpan ke {output_file}")

if __name__ == "__main__":
    input_file = 'telcaf4.csv'
    output_file = 'T4_Inc20_Samp20.csv'
    process_csv_with_averaging(input_file, output_file)