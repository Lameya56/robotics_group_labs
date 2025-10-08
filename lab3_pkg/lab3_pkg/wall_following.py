import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
import numpy as np
from math import radians, tan, atan, cos, sin

def nan_safe_min(a):
    """Returns minimum non-NaN value for mathemathical purposes."""
    # Return min ignoring NaNs; if all NaN, return +inf
    if np.all(np.isnan(a)):
        return np.inf
    return np.nanmin(a)

class WallFollower(Node):
    """Uses PiD control to make the robot locate a left wall and start following around it at a distance."""
    """Initializes the desired_distance from wall, lookahead, and separation angle between left and front view of robot. The PiD gains are initialized as well as the speed of the robot. The scanners and errors are set."""
    def __init__(self):
        super().__init__('wall_follower')

        # === PARAMETERS (FILL SOME) ===
        # Desired left-wall following distance (meters)
        self.d_des = self.declare_parameter('desired_distance', 0.6).value  # e.g., 0.7
        # Lookahead distance L (meters)
        self.lookahead = self.declare_parameter('lookahead', 0.5).value     # e.g., 0.6
        # Beam separation angle theta (deg) for two-range geometry
        self.theta_deg = self.declare_parameter('theta_deg', 25.0).value

        # PID gains
        self.Kp = self.declare_parameter('Kp', 1.2).value  # e.g., 1.8
        self.Ki = self.declare_parameter('Ki', 0.01).value  # e.g., 0.0 (start at 0)
        self.Kd = self.declare_parameter('Kd', 0.6).value  # e.g., 0.2

        # Speed schedule based on |steer|
        self.v_fast = self.declare_parameter('v_fast', 0.35).value
        self.v_med  = self.declare_parameter('v_med',  0.25).value
        self.v_slow = self.declare_parameter('v_slow', 0.15).value
        self.ang_med = radians(15.0)
        self.ang_high = radians(30.0)

        # state
        self.int_err = 0.0
        self.prev_err = 0.0
        self.prev_t = None

        # ROS I/O
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 20)

        self.get_logger().info("Wall follower (left side) started.")

    @staticmethod
    def wrap_to_scan(angle, amin, amax):
        """Wrap an angle into [amin, amax] by adding/subtracting 2π. Gets valid angles for robot's sensor"""
        two_pi = 2.0 * np.pi
        while angle < amin:
            angle += two_pi
        while angle > amax:
            angle -= two_pi
        return angle

    def sample_range(self, msg: LaserScan, angle_rad):
        """
        Return a robust distance reading at a specific bearing by sampling +/- a few indices.
        Gets min valid distance from the robot scan, avoiding any NaN values.
        """
        # Map angle to index
        # LaserScan: angle_min + i*angle_increment
        angle = self.wrap_to_scan(angle_rad, msg.angle_min, msg.angle_max)
        i_center = int(round((angle - msg.angle_min) / msg.angle_increment))
        n = len(msg.ranges)

        window = 2  # sample small window around target index
        idxs = [(i_center + k) % n for k in range(-window, window + 1)]
        vals = np.array(msg.ranges)[idxs]
        return nan_safe_min(vals)

    def scan_cb(self, msg: LaserScan):
        """Using the robot's sensor, find a valid wall to go towards and accounts for fast turning speeds and general forward speed."""
        # Choose two beams for left-side wall following:
        # b at 90 deg (to the left), a at 90 - theta
        # b is facing left exactly, while a is slightly upwards (northwest)
        theta = radians(self.theta_deg)
        a_ang = np.pi/2 - theta   # beam a
        b_ang = np.pi/2           # beam b

        a = self.sample_range(msg, a_ang)
        b = self.sample_range(msg, b_ang)

        # If either is inf (no return), don't command aggressively, stopping invalid oscillations
        if not np.isfinite(a) or not np.isfinite(b):
            self.publish_cmd(0.0, 0.0)
            return

        # Geometry for wall-following:
        # alpha = atan((a*cos(theta) - b) / (a*sin(theta)))
        # D_t   = b * cos(alpha)
        # D_{t+1} = D_t + L * sin(alpha)
        alpha = atan((a * cos(theta) - b) / (a * sin(theta))) if a * sin(theta) != 0 else 0.0
        D_t = b * cos(alpha)
        D_t1 = D_t + self.lookahead * sin(alpha)

        # Distance error (negative if too far from wall), which causes the robot to turn towards the left (searching for a wall).
        err = D_t1 - self.d_des

        # PID update with dt
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self.prev_t is None else max(1e-3, now - self.prev_t)
        self.prev_t = now

        self.int_err = np.clip(self.int_err + err * dt, -1.0, 1.0)
        d_err = (err - self.prev_err) / dt if dt > 0 else 0.0
        self.prev_err = err

        # Steering (angular z) from PID on distance error
        w = self.Kp * err + self.Ki * self.int_err + self.Kd * d_err

        # Speed schedule: slow down for sharp turns
        abs_w = abs(w)
        if abs_w < self.ang_med:
            v = self.v_fast
        elif abs_w < self.ang_high:
            v = self.v_med
        else:
            v = self.v_slow

        # Publish command
        self.publish_cmd(v, w)

    def publish_cmd(self, v, w):
        # Makes the robot move according to what has been scanned by the robot's observation and mathematical expression to move at a certain speed forward, and turn at a certain angle.
        cmd = TwistStamped()
        cmd.twist.linear.x = v
        cmd.twist.angular.z = w
        self.cmd_pub.publish(cmd)

def main():
    # Initializes WallFollower node, and continues to run the node until it is stopped by user interrupt input.
    rclpy.init()
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
