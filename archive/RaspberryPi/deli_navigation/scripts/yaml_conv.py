import csv
import yaml

# Path file input dan output
#bukan node , file untuk conversi trajectory manual
# csv ---> yaml

input_csv = "../data/trajectory_xy_new.csv"
output_yaml = "../data/trajectory.yaml"

# Membaca data CSV dan mengonversi ke YAML
poses = []
with open(input_csv, 'r') as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        pose = {
            'position': {
                'x': float(row['x']),
                'y': float(row['y']),
                'z': 0.0
            },
            'orientation': {
                'x': 0.0,
                'y': 0.0,
                'z': 0.0,
                'w': 1.0
            }
        }
        poses.append(pose)

# Simpan dalam format YAML
with open(output_yaml, 'w') as yamlfile:
    yaml.dump({'poses': poses}, yamlfile)

print(f"File YAML berhasil dibuat: {output_yaml}")
