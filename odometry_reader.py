import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class OdometryReader(Node):
    """
    Reads /odometry/filtered and publishes:
        /robot_pose     Float32MultiArray [x, y, yaw]
        /robot_velocity Float32MultiArray [vx, wz]
    """

    def __init__(self):
        super().__init__('odometry_reader')

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            odom_qos)

        self.pose_pub = self.create_publisher(
            Float32MultiArray, '/robot_pose', 10)

        self.velocity_pub = self.create_publisher(
            Float32MultiArray, '/robot_velocity', 10)

        self.get_logger().info('OdometryReader node started')

    def quaternion_to_yaw(self, x, y, z, w):
        """Full quaternion to yaw conversion."""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg: Odometry):
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        q   = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        vx  = msg.twist.twist.linear.x
        wz  = msg.twist.twist.angular.z

        pose_msg = Float32MultiArray()
        pose_msg.data = [float(x), float(y), float(yaw)]
        self.pose_pub.publish(pose_msg)

        vel_msg = Float32MultiArray()
        vel_msg.data = [float(vx), float(wz)]
        self.velocity_pub.publish(vel_msg)

        self.get_logger().info(
            f'Pose: x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f}° | '
            f'Vel: vx={vx:.3f} wz={wz:.3f}',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdometryReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()