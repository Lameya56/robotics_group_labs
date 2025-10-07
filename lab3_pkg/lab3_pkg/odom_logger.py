import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistStamped
from math import atan2
import csv
import os

def yaw_from_quaternion(qx, qy, qz, qw):
    # Planar yaw extraction (safe for TurtleBot 2D)
    return atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz))

class OdomLogger(Node):
    def __init__(self):
        super().__init__('odom_logger')
	    # Subscribe to odometry
        self.sub = self.create_subscription(Odometry, '/odom', self.cb, 50)
	    # Subscribe to commanded velocities
        self.cmd_sub = self.create_subscription(TwistStamped, '/cmd_vel', self.cmd_cb, 50)

        self.last_cmd_lin_x = 0.0
        self.last_cmd_ang_z = 0.0

        # CSV file path (workspace root). Change if you'd like another location.
        self.csv_path = os.path.expanduser('~/ros2_ws/odom_log.csv')
        # Open CSV for writing (overwrite each run)
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)

        # Header row
        self.writer.writerow([
            't_sec', 'x', 'y', 'yaw',
            'lin_x', 'lin_y', 'lin_z',
            'ang_x', 'ang_y', 'ang_z',
            'cmd_lin_x', 'cmd_ang_z'
        ])

        self.start_time = None
        self.get_logger().info(f"Logging /odom to {self.csv_path}")

    def cb(self, msg: Odometry):
        # time (seconds)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.start_time is None:
            self.start_time = t
        t_rel = t - self.start_time

        # pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

        # twist
        lin = msg.twist.twist.linear
        ang = msg.twist.twist.angular

        # write CSV row
        self.writer.writerow([
            f"{t_rel:.3f}", f"{x:.4f}", f"{y:.4f}", f"{yaw:.4f}",
            f"{lin.x:.4f}", f"{lin.y:.4f}", f"{lin.z:.4f}",
            f"{ang.x:.4f}", f"{ang.y:.4f}", f"{ang.z:.4f}",
	        f"{self.last_cmd_lin_x:.4f}", f"{self.last_cmd_ang_z:.4f}"
        ])

    def cmd_cb(self, msg: TwistStamped):
        self.last_cmd_lin_x = msg.twist.linear.x
        self.last_cmd_ang_z = msg.twist.angular.z

    def destroy_node(self):
        try:
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OdomLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
