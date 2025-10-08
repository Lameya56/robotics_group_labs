import csv, os, math
import matplotlib.pyplot as plt


def _to_float_or_nan(s):   
    s = s.strip() if isinstance(s, str) else s
    return float(s) if s not in (None, "",) else float('nan')

def _any_finite(xs):
    """
    Check if a list contains any finite numbers.
    Args:
        xs (list of float): List of numeric values.    
    Returns:
        bool: True if any value in the list is finite, False otherwise.
    """ 
    return any(math.isfinite(x) for x in xs)

def main():
    """
    Reads logged odometry and EKF-filtered data from a CSV file and plots the
    trajectories against ground truth (if available) for comparison.

    CSV file format (columns):
        t          : time (s)
        odom_x     : x-position from raw odometry
        odom_y     : y-position from raw odometry
        filt_x     : x-position from EKF (/odometry/filtered)
        filt_y     : y-position from EKF (/odometry/filtered)
        gt_x       : x-position from Gazebo ground truth (optional)
        gt_y       : y-position from Gazebo ground truth (optional)
    
    The function generates a plot showing:
        - Odometry trajectory (blue dashed line)
        - EKF-filtered trajectory (red dashed line)
        - Ground truth trajectory (green dotted line, if available)
    """
    path = os.path.expanduser('~/ros2_ws/ekf_compare.csv')
    t = []
    odom_x, odom_y = [], []
    filt_x, filt_y = [], []
    gt_x, gt_y = [], []

    with open(path, 'r') as f:
        r = csv.DictReader(f)
        for row in r:
            t.append(float(row['t']))
            odom_x.append(float(row['odom_x'])); odom_y.append(float(row['odom_y']))
            filt_x.append(float(row['filt_x'])); filt_y.append(float(row['filt_y']))
            gt_x.append(_to_float_or_nan(row.get('gt_x', "")))
            gt_y.append(_to_float_or_nan(row.get('gt_y', "")))

    plt.figure()
    plt.plot(odom_x, odom_y, 'b--', linewidth=3, alpha=0.5, label='Odometry (/odom)')
    plt.plot(filt_x, filt_y, 'r--', linewidth=3, alpha=0.5, label='EKF (/odometry/filtered)')
    if _any_finite(gt_x) and _any_finite(gt_y):
        plt.plot(gt_x, gt_y, 'g:', label='Ground truth', linewidth=4, alpha=0.6)
    plt.gca().set_aspect('equal', 'box')
    plt.xlabel('x (m)'); plt.ylabel('y (m)')
    plt.title('Trajectory Comparison')
    plt.grid(True); plt.legend()
    plt.show()

if __name__ == '__main__':
    main()
