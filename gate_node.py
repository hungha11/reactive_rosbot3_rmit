import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped


class GateNode(Node):
    """
    Safety gate — sole publisher of /cmd_vel.

    Blocks all motion in the hard emergency band so rotation cannot scrape
    a corner sensor along a wall.
    """

    EMERGENCY_DIST   = 0.25
    EMERGENCY_CLEAR  = 0.32
    REAR_EMERGENCY_DIST = 0.18
    REAR_EMERGENCY_CLEAR = 0.28
    WATCHDOG_TIMEOUT = 0.5
    V_LIMIT          = 0.16
    OMEGA_LIMIT      = 0.6

    def __init__(self):
        super().__init__('gate_node')

        self.tof             = [0.9, 0.9, 0.9, 0.9]
        self.last_cmd_time   = self.get_clock().now()
        self.last_brain_cmd  = None
        self.override_active = False
        self.front_latched = False
        self.rear_latched = False

        self.create_subscription(
            TwistStamped, '/cmd_brain',
            self.cmd_brain_callback, 10)

        self.create_subscription(
            Float32MultiArray, '/tof_readings',
            self.tof_callback, 10)

        self.cmd_pub = self.create_publisher(
            TwistStamped, '/cmd_vel', 10)

        self.create_timer(0.05, self.gate_loop)

        self.get_logger().info('GateNode started — sole publisher of /cmd_vel')
        self.get_logger().info(
            f'EMERGENCY_DIST:{self.EMERGENCY_DIST}m '
            f'EMERGENCY_CLEAR:{self.EMERGENCY_CLEAR}m '
            f'REAR_EMERGENCY_DIST:{self.REAR_EMERGENCY_DIST}m '
            f'WATCHDOG:{self.WATCHDOG_TIMEOUT}s'
        )

    def tof_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            self.tof = list(msg.data)

    def cmd_brain_callback(self, msg: TwistStamped):
        self.last_brain_cmd = msg
        self.last_cmd_time  = self.get_clock().now()

    def publish_zero(self, reason: str):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x  = 0.0
        msg.twist.angular.z = 0.0
        self.cmd_pub.publish(msg)

        if not self.override_active:
            self.get_logger().warn(
                f'GATE OVERRIDE → zero | {reason}',
                throttle_duration_sec=1.0
            )
            self.override_active = True

    def gate_loop(self):
        # Watchdog
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.WATCHDOG_TIMEOUT:
            self.publish_zero(f'watchdog: no cmd_brain for {elapsed:.1f}s')
            return

        if self.last_brain_cmd is None:
            self.publish_zero('waiting for cmd_brain')
            return

        v     = self.last_brain_cmd.twist.linear.x
        omega = self.last_brain_cmd.twist.angular.z

        # NaN/Inf guard
        if not math.isfinite(v) or not math.isfinite(omega):
            self.publish_zero(f'NaN/Inf: v={v} ω={omega}')
            return

        fl, fr, rl, rr = self.tof
        front_tof = min(fl, fr)
        rear_tof = min(rl, rr)

        if front_tof < self.EMERGENCY_DIST:
            self.front_latched = True
        if rear_tof < self.REAR_EMERGENCY_DIST:
            self.rear_latched = True

        if self.front_latched and front_tof < self.EMERGENCY_CLEAR and v > 0:
            self.publish_zero(
                f'front emergency: FL:{fl:.2f} FR:{fr:.2f} < clear {self.EMERGENCY_CLEAR}m'
            )
            return

        if self.rear_latched and rear_tof < self.REAR_EMERGENCY_CLEAR and v < 0:
            self.publish_zero(
                f'rear emergency: RL:{rl:.2f} RR:{rr:.2f} < clear {self.REAR_EMERGENCY_CLEAR}m'
            )
            return

        if front_tof >= self.EMERGENCY_CLEAR:
            self.front_latched = False
        if rear_tof >= self.REAR_EMERGENCY_CLEAR:
            self.rear_latched = False

        # Pass through
        if self.override_active:
            self.get_logger().info('Gate override cleared → resuming')
            self.override_active = False

        v = max(-self.V_LIMIT, min(self.V_LIMIT, v))
        omega = max(-self.OMEGA_LIMIT, min(self.OMEGA_LIMIT, omega))

        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x  = float(v)
        msg.twist.angular.z = float(omega)
        self.cmd_pub.publish(msg)

        self.get_logger().info(
            f'v={v:.2f} ω={omega:.2f} | '
            f'FL:{fl:.2f} FR:{fr:.2f} RL:{rl:.2f} RR:{rr:.2f}',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = GateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
