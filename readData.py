import matplotlib.pyplot as plt
import numpy as np

# file path
file_path = '/Users/kaiquefernandes/Documents/TEC/git/tec_simulation/output/imu_dropout/data_log.txt'

# Lists to store data
time = []
pos_x, pos_y, pos_z = [], [], []
rate_x, rate_y, rate_z = [], [], []

# Read and parse the file
with open(file_path, 'r') as file:
    for line in file:
        # Skip header or non-data lines
        if line.startswith('=') or line.startswith('time') or 'AVERAGE' in line:
            continue
        try:
            parts = line.strip().split(',')
            if len(parts) >= 7:
                time.append(float(parts[0]))
                rate_x.append(float(parts[1]) if parts[1] != 'nan' else np.nan)
                rate_y.append(float(parts[2]) if parts[2] != 'nan' else np.nan)
                rate_z.append(float(parts[3]) if parts[3] != 'nan' else np.nan)
                pos_x.append(float(parts[4]) if parts[4] != 'nan' else np.nan)
                pos_y.append(float(parts[5]) if parts[5] != 'nan' else np.nan)
                pos_z.append(float(parts[6]) if parts[6] != 'nan' else np.nan)
        except ValueError:
            continue  # Skip lines with invalid data


# Plotting
plt.figure(figsize=(12, 6))

# Position plot
plt.subplot(2, 1, 1)
plt.plot(time, pos_x, label='pos_x')
plt.plot(time, pos_y, label='pos_y')
plt.plot(time, pos_z, label='pos_z')
plt.title('Position Data')
plt.xlabel('Time')
plt.ylabel('Position')
plt.legend()

# Rate plot
plt.subplot(2, 1, 2)
plt.plot(time, rate_x, label='rate_x')
plt.plot(time, rate_y, label='rate_y')
plt.plot(time, rate_z, label='rate_z')
plt.title('Rate Data')
plt.xlabel('Time')
plt.ylabel('Rate')
plt.legend()

plt.tight_layout()
plt.show()