import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from math import atan2, sqrt, pi

def wrap_to_pi(a):
    # Keep angles in (-pi, pi]
    while a >  pi: a -= 2*pi
    while a <= -pi: a += 2*pi
    return a

class PDWaypoint(Node):
    """
    A PD controller node for following a sequence of 2D waypoints.

    Subscriptions:
        /odom (nav_msgs/Odometry): Robot odometry (pose and twist).

    Publications:
        /cmd_vel (geometry_msgs/TwistStamped): Commanded linear and angular velocities.

    Parameters:
        Kp_lin (float): Proportional gain for linear velocity.
        Kp_ang (float): Proportional gain for angular velocity.
        Kd_ang (float): Derivative gain for angular velocity.
        max_lin (float): Maximum linear velocity.
        max_ang (float): Maximum angular velocity.
        goal_tol (float): Distance threshold to consider a waypoint reached.
    """

    def __init__(self):
        super().__init__('pd_waypoint')

        # === PARAMETERS / GAINS (FILL THESE) ===
        self.Kp_lin = self.declare_parameter('Kp_lin', 0.4).value     # e.g., 0.6
        self.Kp_ang = self.declare_parameter('Kp_ang', 3.0).value     # e.g., 2.0
        self.Kd_ang = self.declare_parameter('Kd_ang', 0.5).value     # e.g., 0.3
        self.max_lin = self.declare_parameter('max_lin', 0.35).value
        self.max_ang = self.declare_parameter('max_ang', 1.2).value
        self.goal_tol = self.declare_parameter('goal_tol', 0.05).value # e.g., 0.08 (meters)

        # Waypoints (a square). You can change/extend these.
        self.waypoints = [
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
        self.wp_idx = 0

        # State
        self.x = 0.0; self.y = 0.0; self.yaw = 0.0
        self.prev_heading_err = 0.0
        self.prev_time = None

        # ROS I/O
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 50)
        self.timer = self.create_timer(0.05, self.control_step)  # 20 Hz

        self.get_logger().info("PD waypoint follower started.")

    def odom_cb(self, msg: Odometry):
        # Pose
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # planar yaw (shortcut like Part 1)
        self.yaw = atan2(2.0*(q.w*q.z), 1.0 - 2.0*(q.z*q.z))

    def control_step(self):
        if self.wp_idx >= len(self.waypoints):
            # Arrived all goals: stop
            self.cmd_pub.publish(TwistStamped())
            return

        # Current goal
        gx, gy = self.waypoints[self.wp_idx]

        # Position & heading errors
        dx = gx - self.x
        dy = gy - self.y
        dist_err = sqrt(dx*dx + dy*dy)

        # Desired heading to goal
        goal_heading = atan2(dy, dx)
        heading_err = wrap_to_pi(goal_heading - self.yaw)

        # Time delta for derivative
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0
        if self.prev_time is not None:
            dt = max(1e-3, now - self.prev_time)  # avoid divide-by-zero
        self.prev_time = now

        # PD on heading (angular velocity)
        d_heading = (heading_err - self.prev_heading_err) / dt if dt > 0.0 else 0.0
        self.prev_heading_err = heading_err

        w = self.Kp_ang * heading_err + self.Kd_ang * d_heading
        v = self.Kp_lin * dist_err

        # Simple saturation
        v = max(min(v, self.max_lin), -self.max_lin)
        w = max(min(w, self.max_ang), -self.max_ang)

        # Slow down when turning sharply (blend)
        turn_scale = max(0.0, 1.0 - abs(heading_err)/pi)  # in [0,1]
        v *= turn_scale

        # Goal reached?
        if dist_err < self.goal_tol:
            self.wp_idx += 1
            self.get_logger().info(f"Reached waypoint {self.wp_idx}/{len(self.waypoints)}")
            # brief stop at each corner
            self.cmd_pub.publish(TwistStamped())
            return

        # Publish
        cmd = TwistStamped()
        cmd.twist.linear.x = v
        cmd.twist.angular.z = w
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = PDWaypoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
