import argparse
import os
import matplotlib.pyplot as plt
import numpy as np
from mcap_ros2.decoder import Decoder
from mcap.reader import make_reader
import pandas as pd
# import module
from tabulate import tabulate

def read_bag_file(file_path, topic_name):
    """
    Membaca data posisi (X, Y) dari file ROS2 bag (MCAP format) untuk topik tertentu.
    """
    x_data = []
    y_data = []
    
    with open(file_path, "rb") as f:
        reader = make_reader(f)
        decoder = Decoder()
        
        for schema, channel, message in reader.iter_messages():
            if channel.topic == topic_name:
                msg = decoder.decode(schema, message)
                x_data.append(msg.data[0])  # Sesuaikan jika berbeda
                y_data.append(msg.data[1])  # Sesuaikan jika berbeda
    
    return x_data, y_data


def compute_ideal_trajectory(start, end, num_points=1300):
    """
    Menghitung trajectory ideal berupa garis lurus dari titik awal ke titik akhir.
    """
    x1, y1 = start
    x2, y2 = end
    t_values = np.linspace(0, 1, num_points)
    x_ideal = x1 + t_values * (x2 - x1)
    y_ideal = y1 + t_values * (y2 - y1)
    return x_ideal, y_ideal

def calculate_errors(x_real, y_real, start, end):
    """
    Menghitung RMSE dan lateral deviation antara trajectory real dan ideal.
    Menangani kasus gerak vertikal (x konstan) dan horizontal (y konstan).
    """
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1

    if dx == 0:  # Garis vertikal
        # Semua nilai x_real diproyeksikan ke x1
        x_projected = np.full_like(x_real, x1)
        y_projected = y_real  # y_real tetap
        lateral_deviation = np.abs(x_real - x1)

    elif dy == 0:  # Garis horizontal
        # Semua nilai y_real diproyeksikan ke y1
        y_projected = np.full_like(y_real, y1)
        x_projected = x_real  # x_real tetap
        lateral_deviation = np.abs(y_real - y1)

    else:  # Garis miring
        d_length_squared = dx**2 + dy**2
        # Proyeksi titik real ke garis ideal
        t_values = ((x_real - x1) * dx + (y_real - y1) * dy) / d_length_squared
        t_values = np.clip(t_values, 0, 1)  # Batasi t-values antara 0 dan 1
        x_projected = x1 + t_values * dx
        y_projected = y1 + t_values * dy
        lateral_deviation = np.abs(dy * (x_real - x1) - dx * (y_real - y1)) / np.sqrt(d_length_squared)

    # Hitung RMSE
    rmse = np.sqrt(np.mean((x_real - x_projected)**2 + (y_real - y_projected)**2))

    # Rata-rata lateral deviation
    average_lateral_deviation = np.mean(lateral_deviation)

    return rmse, average_lateral_deviation

def main():
    # Parameter konfigurasi untuk berbagai jenis gerakan
    movement_args = {
        "lurus_depan": [0, 0, 0, 650],
        "lurus_linearrotasi": [0, 0, 0, 650],
        "lurus_samping": [0, 0, 650, 0],
        "lurus_serong": [0, 0, 460, 460],
        "fullpath": [0, 0, 150, 75, 390, 85, 390, 200, 390, 410, 40, 410],
    }

    tolerances = {
        "cepat": 0.5,
        "sedang": 0.35,
        "lambat": 0.2,
    }

    results_lin = []
    results_path = []
    topic_name = "/sensor/odom_filtered"
    base_folder = os.path.join(os.getcwd(), "bag_file")
    for folder in os.listdir(base_folder):
        if folder.startswith("gerak_"):
            folder_path = os.path.join(base_folder, folder)
            for file in os.listdir(folder_path):
                if file.endswith(".mcap"):
                    file_path = os.path.join(folder_path, file)
                    # Identifikasi jenis gerakan dan toleransi dari nama file
                    for movement, arg in movement_args.items():
                        if movement in file:
                            break
                    else:
                        continue

                    for speed, tolerance in tolerances.items():
                        if speed in file:
                            break
                    # Membaca data dari file ROS2 bag
                    x_data, y_data = read_bag_file(file_path, topic_name)

                    # Membaca waypoints dan membangun trajectory ideal
                    waypoints = [(arg[i], arg[i+1]) for i in range(0, len(arg), 2)]
                    x_ideal_all, y_ideal_all = [], []

                    for i in range(len(waypoints) - 1):
                        start, end = waypoints[i], waypoints[i+1]
                        x_ideal, y_ideal = compute_ideal_trajectory(start, end)
                        x_ideal_all.extend(x_ideal)
                        y_ideal_all.extend(y_ideal)

                    # Hitung error untuk setiap segmen
                    total_rmse = 0
                    for i in range(len(waypoints) - 1):
                        start, end = waypoints[i], waypoints[i+1]

                        # Bounding box segmen untuk filter data real
                        x_min, x_max = min(start[0], end[0]) - tolerance, max(start[0], end[0]) + tolerance
                        y_min, y_max = min(start[1], end[1]) - tolerance, max(start[1], end[1]) + tolerance

                        # x_min, x_max = min(start[0], end[0]), max(start[0], end[0])
                        # y_min, y_max = min(start[1], end[1]), max(start[1], end[1])

                        # Filter data real untuk segmen ini
                        segment_indices = (np.array(x_data) >= x_min) & (np.array(x_data) <= x_max) & \
                                        (np.array(y_data) >= y_min) & (np.array(y_data) <= y_max)
                        x_segment = np.array(x_data)[segment_indices]
                        y_segment = np.array(y_data)[segment_indices]

                        if len(x_segment) > 0 and len(y_segment) > 0:
                            # Hitung RMSE dan lateral deviation untuk segmen ini
                            rmse, lateral_deviation_mean = calculate_errors(x_segment, y_segment, start, end)
                            total_rmse += rmse
                            print(f"Segment {i + 1}: RMSE = {rmse}, Avg Lateral Deviation = {lateral_deviation_mean}")
                        else:
                            print(f"Segment {i + 1}: No data points found in segment bounding box.")
                        if movement == "fullpath":
                            results_path.append({
                                "Jenis Gerakan": movement,
                                "Kecepatan": speed,
                                "RMSE": rmse,
                                "Lateral Deviation": lateral_deviation_mean
                            })
                        else:
                            results_lin.append({
                                "Jenis Gerakan": movement,
                                "Kecepatan": speed,
                                "RMSE": rmse,
                                "Lateral Deviation": lateral_deviation_mean
                            })

                    print(f"Total RMSE: {total_rmse}")
                    # Plot trajectory real dan ideal
                    # plt.figure(figsize=(8, 8))
                    # plt.plot(x_data, y_data, label="Trajectory Real", color="blue", alpha=0.7)
                    # plt.plot(x_ideal_all, y_ideal_all, label="Trajectory Ideal", color="red", linestyle="--", alpha=0.7)
                    # plt.xlabel("X Position")
                    # plt.ylabel("Y Position")
                    # plt.title("Robot Trajectory with Multi-Segment Ideal Path")
                    # plt.legend()
                    # plt.grid()
                    # plt.axis("equal")
                    # plt.show()
    # Konversi hasil ke DataFrame untuk tabel
    df_results = pd.DataFrame(results_lin)
    df_results["RMSE"] = df_results["RMSE"].astype(float)
    print(tabulate(df_results))
    df_results.to_csv("trajectory_linear_results.csv", sep=';',index=False, float_format='%.4f')
    df_results = pd.DataFrame(results_path)
    df_results["RMSE"] = df_results["RMSE"].astype(float)
    print(tabulate(df_results))
    df_results.to_csv("trajectory_full_results.csv", sep=';',index=False, float_format='%.4f')

if __name__ == "__main__":
    main()
