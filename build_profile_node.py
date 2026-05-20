#!/usr/bin/env python3
"""
build_profile_node.py — Fuses LiDAR + ToF + camera into the final /profile.

What changed vs. the original:
    + Subscribes to /camera_profile (from depth_reader.py).
    + Fuses camera bins with the "min wins, only lower" rule.
    + Camera IS allowed to lower the central front zone — unlike FL/FR ToFs.
      Reason: the camera is the only sensor that can see low/overhanging
      obstacles directly ahead, so it must own the front zone.

Inputs:
    /lidar_profile      Float32MultiArray  360 floats (LiDAR, hub-plane only)
    /tof_readings       Float32MultiArray  4 floats   [fl, fr, rl, rr]
    /camera_profile     Float32MultiArray  360 floats (low + overhang + drop-off)   ← NEW

Output:
    /profile            Float32MultiArray  360 floats (final fused profile)

Fusion order:
    1. Start from /lidar_profile.
    2. Camera lowers ANY bin (including front protect zone).
    3. FL / FR ToFs lower their projected bins, but NOT the front protect zone.
    4. RL / RR ToFs lower their projected rear bins (no protect zone).

Publishing:
    Triggered on every LiDAR update (preserves original cadence and keeps
    decision rate paced by the SLAMTEC). Camera/ToF data are buffered between
    LiDAR ticks; this is acceptable because LiDAR runs at 10–15 Hz and the
    robot's V_MAX is 0.18 m/s (worst-case detection latency ~1 cm of travel).
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class BuildProfileNode(Node):

    TOF_SENSORS = {
        'fl': {'center':  12.5,  'idx': 0},
        'fr': {'center': -12.5,  'idx': 1},
        'rl': {'center':  167.5, 'idx': 2},
        'rr': {'center': -167.5, 'idx': 3},
    }
    TOF_HALF_FOV      = 12.5
    FRONT_PROTECT_DEG = 10
    MAX_RANGE         = 12.0
    TOF_MAX           = 0.9
    CAMERA_MAX        = 3.0   # must match DepthReader.MAX_RANGE

    def __init__(self):
        super().__init__('build_profile_node')

        # State
        self.lidar_profile  = [self.MAX_RANGE]  * 360
        self.camera_profile = [self.CAMERA_MAX] * 360
        self.tof_readings   = [self.TOF_MAX]    * 4
        self.has_camera = False
        self.has_lidar  = False

        # Bins where FL/FR ToFs are NOT allowed to lower the profile
        # (the camera handles the front zone for low/overhanging obstacles).
        self.front_protect = {
            angle % 360
            for angle in range(-self.FRONT_PROTECT_DEG, self.FRONT_PROTECT_DEG + 1)
        }

        # Pre-compute the bin set each ToF projects into
        self.tof_bins = {}
        for name, cfg in self.TOF_SENSORS.items():
            center = cfg['center']
            bins = set()
            for angle in range(
                int(math.floor(center - self.TOF_HALF_FOV)),
                int(math.ceil(center + self.TOF_HALF_FOV)) + 1,
            ):
                bins.add(angle % 360)
            self.tof_bins[name] = list(bins)

        for name in ('fl', 'fr', 'rl', 'rr'):
            b = sorted(self.tof_bins[name])
            self.get_logger().info(
                f'ToF {name.upper()} bins: {b[0]}°→{b[-1]}° ({len(b)} bins)'
            )
        self.get_logger().info(
            f'Front protect zone (FL/FR ToFs excluded, camera allowed): '
            f'{sorted(self.front_protect)}°'
        )

        # Subscriptions
        self.create_subscription(
            Float32MultiArray, '/lidar_profile',
            self.lidar_callback, 10)
        self.create_subscription(
            Float32MultiArray, '/camera_profile',
            self.camera_callback, 10)
        self.create_subscription(
            Float32MultiArray, '/tof_readings',
            self.tof_callback, 10)

        self.profile_pub = self.create_publisher(
            Float32MultiArray, '/profile', 10)

        self.get_logger().info('BuildProfileNode started (LiDAR + ToF + camera)')

    # ── Callbacks ────────────────────────────────────────────────────────
    def lidar_callback(self, msg: Float32MultiArray):
        if len(msg.data) != 360:
            return
        self.lidar_profile = list(msg.data)
        self.has_lidar = True
        self.fuse_and_publish()

    def camera_callback(self, msg: Float32MultiArray):
        if len(msg.data) != 360:
            return
        self.camera_profile = list(msg.data)
        self.has_camera = True

    def tof_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            self.tof_readings = list(msg.data)

    # ── Fusion ───────────────────────────────────────────────────────────
    def fuse_and_publish(self):
        # 1. LiDAR baseline
        profile = list(self.lidar_profile)

        # 2. Camera — lowers any bin (no front protect exclusion)
        for i in range(360):
            cam_val = self.camera_profile[i]
            if cam_val < profile[i]:
                profile[i] = cam_val

        # 3. ToFs — FL/FR excluded from front protect zone
        for i, name in enumerate(('fl', 'fr', 'rl', 'rr')):
            tof_dist = self.tof_readings[i]
            if tof_dist >= self.TOF_MAX:
                continue
            for b in self.tof_bins[name]:
                if name in ('fl', 'fr') and b in self.front_protect:
                    continue
                if tof_dist < profile[b]:
                    profile[b] = tof_dist

        out = Float32MultiArray()
        out.data = profile
        self.profile_pub.publish(out)

        self.get_logger().info(
            f'Profile — '
            f'F:{profile[0]:.2f} L:{profile[90]:.2f} '
            f'R:{profile[270]:.2f} Rear:{profile[180]:.2f} | '
            f'ToF FL:{self.tof_readings[0]:.2f} FR:{self.tof_readings[1]:.2f} '
            f'RL:{self.tof_readings[2]:.2f} RR:{self.tof_readings[3]:.2f} | '
            f'cam:{"ok" if self.has_camera else "MISSING"}',
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = BuildProfileNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()