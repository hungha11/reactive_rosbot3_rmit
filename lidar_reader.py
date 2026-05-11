## lidar_reader.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
import numpy as np


class LidarReader(Node):
    """
    Reads /scan_filtered and builds a 360-bin distance profile.

    Each bin covers 1 degree. Uses median per bin to reject
    single-ray spikes from dust or glare.

    Input:
        /scan_filtered   LaserScan

    Output:
        /lidar_profile   Float32MultiArray  360 floats (meters)
                         index 0   = 0°   (forward)
                         index 90  = 90°  (left)
                         index 180 = 180° (rear)
                         index 270 = 270° = -90° (right)
    """

    MAX_RANGE   = 12.0   # meters — discard beyond this
    MIN_RANGE   = 0.15   # meters — discard closer than this

    def __init__(self):
        super().__init__('lidar_reader')

        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.create_subscription(
            LaserScan,
            '/scan_filtered',
            self.scan_callback,
            lidar_qos
        )

        self.profile_pub = self.create_publisher(
            Float32MultiArray,
            '/lidar_profile',
            10
        )

        self.get_logger().info('LidarReader started')

    def scan_callback(self, msg: LaserScan):
        """
        Called on every LiDAR scan.
        Builds 360-bin profile using median per bin.
        """

        # Accumulate rays per bin
        bins = [[] for _ in range(360)]

        angle     = msg.angle_min
        angle_inc = msg.angle_increment

        for r in msg.ranges:
            # Validate reading
            if math.isfinite(r) and self.MIN_RANGE < r < self.MAX_RANGE:
                # Convert angle to bin index (0° = forward)
                deg = math.degrees(angle) % 360
                idx = int(round(deg)) % 360
                bins[idx].append(r)

            angle += angle_inc

        # Build profile: median per bin, MAX_RANGE if empty
        profile = [
            float(np.median(b)) if b else self.MAX_RANGE
            for b in bins
        ]

        msg_out = Float32MultiArray()
        msg_out.data = profile
        self.profile_pub.publish(msg_out)

        self.get_logger().info(
            f'Profile published — '
            f'F:{profile[0]:.2f} L:{profile[90]:.2f} '
            f'R:{profile[270]:.2f} Rear:{profile[180]:.2f}',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()