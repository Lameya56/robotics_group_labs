import csv
import os
import matplotlib.pyplot as plt

def main():
    """
    Reads logged PD controller trajectory data from a CSV file and plots
    it against the commanded square path (waypoints) for comparison.

    CSV file format:
        x : X position of the robot (meters)
        y : Y position of the robot (meters)

    The function generates a plot showing:
        - The commanded path (red dashed line)
        - The actual trajectory from the PD controller (blue dashed line)
    """
    csv_path = os.path.expanduser('~/ros2_ws/odom_log.csv')
    x, y = [], []

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            x.append(float(row['x']))
            y.append(float(row['y']))

    waypoints = [
        (0.2, 0.0),
        (0.4, 0.0),
        (0.6, 0.0),
        (0.8, 0.0),
        (0.8, 0.2),
        (0.8, 0.4),
        (0.8, 0.6),
        (0.8, 0.8),
        (0.6, 0.8),
        (0.4, 0.8),
        (0.2, 0.8),
        (0.0, 0.8),
        (0.0, 0.6),
        (0.0, 0.4),
        (0.0, 0.2),
        (0.0, 0.0),
    ]

    # Separate waypoints into x and y lists
    wp_x = [wp[0] for wp in waypoints]
    wp_y = [wp[1] for wp in waypoints]

    plt.figure()
    plt.plot(wp_x, wp_y, 'r--', linewidth=2, markersize=8, label='Commanded Path')
    plt.plot(x, y, 'b--', linewidth=2, label='PD Trajectory')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('PD Trajectory vs Commanded Path')
    plt.grid(True)
    plt.legend()

    plt.show()

if __name__ == '__main__':
    main()
