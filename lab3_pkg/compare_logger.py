import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from math import atan2
import csv, os

def yaw_from_quat(q):
    # planar yaw shortcut
    """
    Convert a quaternion to planar yaw (rotation about Z-axis).

    Args:
        q (geometry_msgs.msg.Quaternion): Quaternion from a ROS message.

    Returns:
        float: Yaw angle in radians.
    """
    return atan2(2.0*(q.w*q.z), 1.0 - 2.0*(q.z*q.z))

class CompareLogger(Node):
    """
    ROS2 node that logs and compares the trajectories from:
        - /odom (wheel odometry)
        - /odometry/filtered (EKF filtered)
        - /gazebo/model_states (ground truth)
    Outputs the data to a CSV file for later plotting.
    """
    def __init__(self):
        super().__init__('compare_logger')

        self.model_name = self.declare_parameter('model_name', 'turtlebot3_burger').value
        # e.g., 'turtlebot3_burger' (check with: ros2 topic echo /gazebo/robot_description --once)

        self.odom = None
        self.filt = None
        self.gt = None

        self.create_subscription(Odometry, '/odom', self.odom_cb, 20)
        self.create_subscription(Odometry, '/odometry/filtered', self.filt_cb, 20)
        self.create_subscription(ModelStates, '/gazebo/model_states', self.ms_cb, 5)

        self.csv_path = os.path.expanduser('~/ros2_ws/ekf_compare.csv')
        self.csv = open(self.csv_path, 'w', newline='')
        self.w = csv.writer(self.csv)
        self.w.writerow([
            't', 
            'odom_x','odom_y','odom_yaw',
            'filt_x','filt_y','filt_yaw',
            'gt_x','gt_y','gt_yaw'
        ])
        self.t0 = None
        self.get_logger().info(f"Logging to {self.csv_path}")

        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz

    def odom_cb(self, msg: Odometry):
        """
        Callback for /odom topic. Stores the latest odometry data.

        Args:
            msg (Odometry): Odometry message.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = yaw_from_quat(q)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom = (t, x, y, yaw)

    def filt_cb(self, msg: Odometry):
        """
        Callback for /odometry/filtered topic. Stores the latest EKF-filtered odometry.

        Args:
            msg (Odometry): Filtered odometry message.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = yaw_from_quat(q)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.filt = (t, x, y, yaw)

    def ms_cb(self, msg: ModelStates):
        """
        Callback for /gazebo/model_states topic. Stores ground truth pose.

        Args:
            msg (ModelStates): Gazebo model states message.
        """
        if self.model_name in msg.name:
            idx = msg.name.index(self.model_name)
            pose = msg.pose[idx]
            q = pose.orientation
            yaw = yaw_from_quat(q)
            # Gazebo ModelStates lacks a header stamp; approximate with ROS clock
            t = self.get_clock().now().nanoseconds * 1e-9
            self.gt = (t, pose.position.x, pose.position.y, yaw)

    def tick(self):
        """
        Timer callback that writes the latest odometry, filtered, and ground truth
        data to the CSV file.
        """
        if self.odom is None or self.filt is None:
            return

        t = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = t
        trel = t - self.t0

        row = [f"{trel:.3f}"]
        row += [f"{v:.4f}" for v in self.odom[1:4]]
        row += [f"{v:.4f}" for v in self.filt[1:4]]

        # write GT if we have it; otherwise write blanks/NaNs
        if self.gt is not None:
            row += [f"{v:.4f}" for v in self.gt[1:4]]
        else:
            row += ["", "", ""]  # or "nan", "nan", "nan"

        self.w.writerow(row)
        self.csv.flush()

    def destroy_node(self):
        """
        Close CSV file before destroying the node.
        """
        try:
            self.csv.close()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = CompareLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
