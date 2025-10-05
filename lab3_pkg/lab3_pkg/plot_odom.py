import csv
import os
import matplotlib.pyplot as plt
import math

def read_csv(path):
    xs, ys, ts = [], [], []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                xs.append(float(row['x']))
                ys.append(float(row['y']))
                ts.append(float(row['t_sec']))
            except Exception:
                continue
    return ts, xs, ys

def plot_path(xs, ys, L=None):
    plt.figure(figsize=(6,6))
    plt.plot(xs, ys, linewidth=2, label='Odometry path (/odom)')
    plt.axis('equal')
    if L is not None:
        # overlay an ideal square anchored at the start position
        x0, y0 = xs[0], ys[0]
        sqx = [x0, x0+L, x0+L, x0, x0]
        sqy = [y0, y0, y0+L, y0+L, y0]
        plt.plot(sqx, sqy, linestyle='--', linewidth=1.5, label=f'Commanded square L={L} m')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title('Odometry Trajectory')
    plt.grid(True)
    plt.legend()
    plt.show()

def main():
    csv_path = os.path.expanduser('~/ros2_ws/odom_log.csv')
    if not os.path.exists(csv_path):
        print(f"No CSV found at {csv_path}. Run odom_logger first.")
        return
    ts, xs, ys = read_csv(csv_path)
    if len(xs) < 2:
        print("Not enough points in CSV to plot.")
        return

    # Optional: set L to the commanded side length from your square node (e.g., 0.5)
    L = 0.5
    plot_path(xs, ys, L=L)

if __name__ == '__main__':
    main()
