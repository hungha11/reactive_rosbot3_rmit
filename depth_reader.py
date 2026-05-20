#!/usr/bin/env python3
"""
depth_reader.py — OAK-D depth → 360-bin obstacle profile (virtual scan).

Same idea as the standard ROS depthimage_to_laserscan package, inlined so the
output matches the Float32MultiArray contract the rest of the stack already
uses.

Algorithm (per depth frame, ~30 Hz):
  1. Slice a horizontal band of pixel rows around the optical center.
  2. For each column in that band, find the closest valid depth.
  3. Convert column → bearing angle using camera intrinsics:
        theta_robot = -atan2(col - cx, fx)   (sign flip: image-right → robot-right)
  4. Bin into the same 360-bin polar grid the LiDAR uses (0=fwd, 90=left).
  5. Take min per bin and publish.

What this DOES detect:
  - Anything in the forward ~66° (OAK-D HFOV) at distances 0.2-3 m, within the
    pixel-band's vertical slice. With band ±20 rows and the OAK-D Pro
    intrinsics you have (fx~618, cy~218, cam at ~22 cm height), the band
    covers roughly 0.12-0.32 m height at 1-3 m distance. Catches things
    just below and just above the LiDAR plane.

What this DOES NOT detect (intentional — the rest of the stack handles these):
  - Floor-level obstacles right in front (< 1 m, < 10 cm tall):
        FL/FR ToFs handle that zone.
  - Overhangs above ~35 cm:
        Mostly irrelevant for ROSbot's collision envelope.
  - Drop-offs:
        Unreliable from a single forward camera without ground-plane fitting.

Why the floor doesn't false-positive: with the band sitting at cy ± 20 rows,
the floor first projects to a row inside the band only at distance
fy * h_cam / 20 ~= 6.8 m, which is past the 3 m cap. The band literally
cannot see the floor in the configured range.

Calibration: ONE knob, BAND_ROWS_ABOVE / BAND_ROWS_BELOW. No TF, no extrinsics.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray


class DepthReader(Node):

    # ── Pixel band (the only real knob) ──────────────────────────────────
    # Rows above and below the optical center to scan for obstacles.
    # Keep small to avoid the floor showing up at long ranges.
    BAND_ROWS_ABOVE = 20
    BAND_ROWS_BELOW = 20

    # ── Depth filter ─────────────────────────────────────────────────────
    MIN_DEPTH = 0.20    # OAK-D Pro reliable min
    MAX_DEPTH = 3.0     # match planner D_MAX

    # ── Output ───────────────────────────────────────────────────────────
    N_BINS = 360
    MAX_RANGE = 3.0     # bins with no obstacle get this value

    def __init__(self):
        super().__init__('depth_reader')

        # Intrinsics (cached from camera_info)
        self.fx = self.cx = self.cy = None
        self.img_w = self.img_h = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )

        self.create_subscription(
            CameraInfo, '/oak/stereo/camera_info',
            self.camera_info_callback, sensor_qos)
        self.create_subscription(
            Image, '/oak/stereo/image_raw',
            self.depth_callback, sensor_qos)

        self.profile_pub = self.create_publisher(
            Float32MultiArray, '/camera_profile', 10)

        self.get_logger().info(
            f'DepthReader started (virtual scan) — '
            f'band:+/-{self.BAND_ROWS_ABOVE}/{self.BAND_ROWS_BELOW} rows '
            f'depth:[{self.MIN_DEPTH:.2f},{self.MAX_DEPTH:.2f}]m'
        )

    # ── Setup ────────────────────────────────────────────────────────────
    def camera_info_callback(self, msg: CameraInfo):
        if self.fx is not None:
            return
        self.fx = float(msg.k[0])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.img_w = msg.width
        self.img_h = msg.height
        self.get_logger().info(
            f'Intrinsics — fx:{self.fx:.1f} cx:{self.cx:.1f} cy:{self.cy:.1f} '
            f'{self.img_w}x{self.img_h}'
        )

    # ── Main loop ────────────────────────────────────────────────────────
    def depth_callback(self, msg: Image):
        if self.fx is None:
            return  # wait for intrinsics
        try:
            depth = self._decode_depth(msg)
        except Exception as e:
            self.get_logger().warn(
                f'depth decode failed: {e}', throttle_duration_sec=2.0)
            return
        profile = self._project(depth)
        self._publish(profile)

    def _decode_depth(self, msg: Image) -> np.ndarray:
        """Return depth in meters as float32 (H, W)."""
        if msg.encoding in ('16UC1', 'mono16'):
            d = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            return d.astype(np.float32) / 1000.0  # mm -> m
        if msg.encoding == '32FC1':
            return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        raise ValueError(f'unsupported depth encoding: {msg.encoding}')

    def _project(self, depth: np.ndarray) -> np.ndarray:
        """
        Depth image → 360-bin obstacle profile.
        Closest-depth-per-column inside a narrow band around the optical center.
        """
        H, W = depth.shape

        # 1. Slice the band of rows around the optical center
        cy_int = int(self.cy)
        row_min = max(0, cy_int - self.BAND_ROWS_ABOVE)
        row_max = min(H, cy_int + self.BAND_ROWS_BELOW + 1)
        band = depth[row_min:row_max, :]   # (band_H, W)

        # 2. Find min valid depth per column
        valid = (band > self.MIN_DEPTH) & (band < self.MAX_DEPTH) & np.isfinite(band)
        masked = np.where(valid, band, np.inf)
        col_min = masked.min(axis=0)        # (W,)
        has_obstacle = col_min < np.inf
        if not has_obstacle.any():
            return np.full(self.N_BINS, self.MAX_RANGE, dtype=np.float32)

        # 3. Column index → bearing angle from forward
        #    In camera optical frame, +X is image-right. The robot's right is
        #    its -Y. lidar_reader convention: 0°=forward, +90°=left, +270°=right.
        #    So an image-right pixel must map to a NEGATIVE robot angle.
        cols = np.arange(W, dtype=np.float32)[has_obstacle]
        cam_angles_deg = np.degrees(np.arctan2(cols - self.cx, self.fx))
        robot_angles_deg = -cam_angles_deg     # image-right -> robot-right

        # 4. Bin and scatter-min
        bins = np.mod(robot_angles_deg.astype(np.int32), self.N_BINS)
        ranges = col_min[has_obstacle].astype(np.float32)
        profile = np.full(self.N_BINS, self.MAX_RANGE, dtype=np.float32)
        np.minimum.at(profile, bins, ranges)
        return profile

    # ── Publish ──────────────────────────────────────────────────────────
    def _publish(self, profile: np.ndarray):
        msg = Float32MultiArray()
        msg.data = profile.tolist()
        self.profile_pub.publish(msg)

        n_hit = int((profile < self.MAX_RANGE).sum())
        self.get_logger().info(
            f'Camera profile — '
            f'F:{float(profile[0]):.2f} '
            f'L:{float(profile[90]):.2f} '
            f'R:{float(profile[270]):.2f} | '
            f'bins_with_obstacle:{n_hit}/{self.N_BINS}',
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = DepthReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()