## tof_reader.py


import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class ToFReaderV2(Node):
    """
    Reads 4 ToF sensors and publishes raw distances.
    No state classification — that's handled by build_profile_node.

    Input:
        /range/fl, /range/fr, /range/rl, /range/rr   LaserScan (ranges[0])

    Output:
        /tof_readings   Float32MultiArray [fl, fr, rl, rr]  (meters)
                        Invalid readings → 0.9 (sensor max range = clear)
    """

    MAX_RANGE = 0.9   # ToF sensor physical maximum

    def __init__(self):
        super().__init__('tof_reader')

        self.distances = {'fl': self.MAX_RANGE,
                          'fr': self.MAX_RANGE,
                          'rl': self.MAX_RANGE,
                          'rr': self.MAX_RANGE}

        tof_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        for sensor_id in ['fl', 'fr', 'rl', 'rr']:
            self.create_subscription(
                LaserScan,
                f'/range/{sensor_id}',
                lambda msg, sid=sensor_id: self.tof_callback(msg, sid),
                tof_qos
            )

        self.pub = self.create_publisher(
            Float32MultiArray, '/tof_readings', 10)

        self.create_timer(0.1, self.publish_readings)

        self.get_logger().info('ToFReaderV2 started')

    def tof_callback(self, msg: LaserScan, sensor_id: str):
        if not msg.ranges:
            return
        raw = msg.ranges[0]
        if math.isfinite(raw) and 0.0 < raw <= self.MAX_RANGE:
            self.distances[sensor_id] = raw
        else:
            self.distances[sensor_id] = self.MAX_RANGE

    def publish_readings(self):
        msg = Float32MultiArray()
        msg.data = [
            self.distances['fl'],
            self.distances['fr'],
            self.distances['rl'],
            self.distances['rr'],
        ]
        self.pub.publish(msg)

        self.get_logger().info(
            f'ToF — FL:{self.distances["fl"]:.2f} '
            f'FR:{self.distances["fr"]:.2f} '
            f'RL:{self.distances["rl"]:.2f} '
            f'RR:{self.distances["rr"]:.2f}',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = ToFReaderV2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()