## build_profile_node.py

import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class BuildProfileNode(Node):
    """
    Merges LiDAR profile and ToF readings into final obstacle profile.

    Inputs:
        /lidar_profile   Float32MultiArray  360 floats (from lidar_reader)
        /tof_readings    Float32MultiArray  [fl, fr, rl, rr] (from tof_reader)

    Output:
        /profile         Float32MultiArray  360 floats (final fused profile)

    Fusion rule:
        ToF only LOWERS a bin — never raises it.
        FL and FR do not lower the central front protection zone; the planner
        handles low front obstacles separately.
    """

    TOF_SENSORS = {
        'fl': {'center':  12.5,   'idx': 0},
        'fr': {'center': -12.5,   'idx': 1},
        'rl': {'center':  167.5,  'idx': 2},
        'rr': {'center': -167.5,  'idx': 3},
    }
    TOF_HALF_FOV = 12.5
    FRONT_PROTECT_DEG = 10

    MAX_RANGE = 12.0
    TOF_MAX   = 0.9

    def __init__(self):
        super().__init__('build_profile_node')

        self.lidar_profile = [self.MAX_RANGE] * 360
        self.tof_readings  = [self.TOF_MAX]   * 4
        self.front_protect = {
            angle % 360
            for angle in range(-self.FRONT_PROTECT_DEG, self.FRONT_PROTECT_DEG + 1)
        }

        # Pre-compute bin sets for each ToF sensor
        self.tof_bins = {}
        for name, cfg in self.TOF_SENSORS.items():
            center = cfg['center']
            bins   = set()
            start  = int(math.floor(center - self.TOF_HALF_FOV))
            end    = int(math.ceil(center  + self.TOF_HALF_FOV))
            for angle in range(start, end + 1):
                bins.add(angle % 360)
            self.tof_bins[name] = list(bins)

        # Log bin ranges
        for name in ['fl', 'fr', 'rl', 'rr']:
            b = sorted(self.tof_bins[name])
            self.get_logger().info(
                f'ToF {name.upper()} bins: {b[0]}°→{b[-1]}° ({len(b)} bins)'
            )
        self.get_logger().info(
            f'Front protect zone: {sorted(self.front_protect)}'
        )

        self.create_subscription(
            Float32MultiArray, '/lidar_profile',
            self.lidar_callback, 10)

        self.create_subscription(
            Float32MultiArray, '/tof_readings',
            self.tof_callback, 10)

        self.profile_pub = self.create_publisher(
            Float32MultiArray, '/profile', 10)

        self.get_logger().info('BuildProfileNode started')

    def lidar_callback(self, msg: Float32MultiArray):
        if len(msg.data) != 360:
            return
        self.lidar_profile = list(msg.data)
        self.fuse_and_publish()

    def tof_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            self.tof_readings = list(msg.data)

    def fuse_and_publish(self):
        profile = list(self.lidar_profile)

        sensor_names = ['fl', 'fr', 'rl', 'rr']
        for i, name in enumerate(sensor_names):
            tof_dist = self.tof_readings[i]

            if tof_dist >= self.TOF_MAX:
                continue

            for b in self.tof_bins[name]:
                if name in ('fl', 'fr') and b in self.front_protect:
                    continue
                profile[b] = min(profile[b], tof_dist)

        msg = Float32MultiArray()
        msg.data = profile
        self.profile_pub.publish(msg)

        self.get_logger().info(
            f'Profile — '
            f'F:{profile[0]:.2f} L:{profile[90]:.2f} '
            f'R:{profile[270]:.2f} Rear:{profile[180]:.2f} | '
            f'ToF FL:{self.tof_readings[0]:.2f} FR:{self.tof_readings[1]:.2f} '
            f'RL:{self.tof_readings[2]:.2f} RR:{self.tof_readings[3]:.2f}',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = BuildProfileNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
