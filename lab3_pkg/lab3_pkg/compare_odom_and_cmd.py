import csv
import os
import matplotlib.pyplot as plt

def main():
    """
    Reads odometry and command velocity data from a CSV file and generates
    plots for linear and angular velocity tracking over time.

    CSV file format:
        t_sec       : Time in seconds
        lin_x       : Linear velocity measured from odometry
        ang_z       : Angular velocity measured from odometry
        cmd_lin_x   : Commanded linear velocity
        cmd_ang_z   : Commanded angular velocity

    The function produces two figures:
        1. Linear velocity vs. time
        2. Angular velocity vs. time
    """
    csv_path = os.path.expanduser('~/ros2_ws/odom_log.csv')
    t, lin_odom, ang_odom, lin_cmd, ang_cmd = [], [], [], [], []

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row['t_sec']))
            lin_odom.append(float(row['lin_x']))
            ang_odom.append(float(row['ang_z']))
            lin_cmd.append(float(row['cmd_lin_x']))
            ang_cmd.append(float(row['cmd_ang_z']))

    # Linear velocities
    plt.figure()
    plt.plot(t, lin_cmd, 'r-', label='cmd_lin_x')
    plt.plot(t, lin_odom, 'b--', label='odom_lin_x')
    plt.xlabel('Time (s)')
    plt.ylabel('Linear velocity (m/s)')
    plt.title('Linear Velocity Tracking')
    plt.grid(True)
    plt.legend()

    # Angular velocities
    plt.figure()
    plt.plot(t, ang_cmd, 'r-', label='cmd_ang_z')
    plt.plot(t, ang_odom, 'b--', label='odom_ang_z')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular velocity (rad/s)')
    plt.title('Angular Velocity Tracking')
    plt.grid(True)
    plt.legend()

    plt.show()

if __name__ == '__main__':
    main()
