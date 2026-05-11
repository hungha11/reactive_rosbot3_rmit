## decide_node.py — Follow The Gap (FTG) reactive obstacle avoidance
##
## Reference: f1t-ftg (idea/f1t-ftg/) by Petr Stepan et al.
## Adapted to /profile (360-bin) + /tof_readings; no TF; no global goal.
##
## PIPELINE (per /profile update):
##   1. For each reachable forward direction θ:
##      score = min_profile(θ ± SECTOR_HALF) + forward_bias
##   2. Best direction → θ_gap
##   3. Blend: θ_final = (α/d_min · θ_gap) / (α/d_min + 1)
##   4. Clamp θ_final to the reachable forward arc, then arc_cmd(...)
##   5. Speed gated by forward profile clearance and ToF FL/FR

import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped


class DecideNode(Node):

    # ── Robot geometry ───────────────────────────────────────────────────
    R          = 0.167   # m — half-width

    # ── FTG sector scoring ───────────────────────────────────────────────
    D_MAX        = 3.0   # m — profile max (no-obstacle sentinel)
    SECTOR_HALF  = 20    # deg — half-width of sector for clearance scoring
    FORWARD_BIAS = 0.25  # m — virtual clearance bonus for forward-ish dirs
    GAP_WEIGHT   = 80.0  # α — gap-center vs. forward blending
    SEARCH_DEG   = 60    # deg — keep goals in a forward corridor
    FRONT_HALF    = 10    # deg — true forward stop sector
    SIDE_CLEAR    = 0.40  # m — begin biasing away from close side/corner walls

    # ── Steering limit ───────────────────────────────────────────────────
    MAX_STEER  = math.radians(45)

    # ── Speed control ────────────────────────────────────────────────────
    FRONT_STOP   = 0.22  # m — planner stop distance before the hard gate
    TOF_PAIR_STOP = 0.28 # m — trust front ToFs only when both corners agree
    DVEL_SAFE    = 0.75  # m — distance at which speed starts reducing
    V_MAX        = 0.18  # m/s
    OMEGA_MAX    = 0.6   # rad/s
    ACCEL_LIMIT  = 0.08  # m/s^2
    OMEGA_ACCEL  = 0.5   # rad/s^2
    ESCAPE_ENTER = 0.28  # m — front corner contact/stuck threshold
    ESCAPE_EXIT  = 0.36  # m — stay in escape until the corner is clear
    ESCAPE_MIN_TIME = 1.0 # s — do not quit escape after one noisy sample
    REAR_CLEAR   = 0.30  # m — enough room for a small reverse escape
    V_ESCAPE     = -0.05 # m/s
    OMEGA_ESCAPE = 0.35  # rad/s

    def __init__(self):
        super().__init__('decide_node')

        self.profile = np.full(360, self.D_MAX, dtype=np.float32)
        self.tof_fl  = self.D_MAX   # FL ToF distance (m)
        self.tof_fr  = self.D_MAX   # FR ToF distance (m)
        self.tof_rl  = self.D_MAX
        self.tof_rr  = self.D_MAX
        self.last_v = 0.0
        self.last_omega = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.escape_side = None
        self.escape_until_ns = 0
        self.low_obstacle_side = None
        self.low_obstacle_until_ns = 0

        self.create_subscription(
            Float32MultiArray, '/profile',
            self.profile_callback, 10)

        self.create_subscription(
            Float32MultiArray, '/tof_readings',
            self.tof_callback, 10)

        self.cmd_pub = self.create_publisher(
            TwistStamped, '/cmd_brain', 10)

        self.get_logger().info(
            f'DecideNode (FTG) — '
            f'R={self.R}m  SECTOR={self.SECTOR_HALF}°  SEARCH=±{self.SEARCH_DEG}°  '
            f'MAX_STEER={math.degrees(self.MAX_STEER):.0f}°  '
            f'V_MAX={self.V_MAX}m/s  TOF_PAIR={self.TOF_PAIR_STOP}m  '
            f'ESCAPE={self.ESCAPE_ENTER}->{self.ESCAPE_EXIT}m'
        )

    # ── ROS callbacks ─────────────────────────────────────────────────────

    def tof_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            # 0.9m = max range (clear); smaller = obstacle present
            self.tof_fl = max(0.01, float(msg.data[0]))
            self.tof_fr = max(0.01, float(msg.data[1]))
            self.tof_rl = max(0.01, float(msg.data[2]))
            self.tof_rr = max(0.01, float(msg.data[3]))

    def profile_callback(self, msg: Float32MultiArray):
        if len(msg.data) != 360:
            return
        self.profile = np.array(msg.data, dtype=np.float32)
        bad = ~np.isfinite(self.profile) | (self.profile < 0)
        self.profile[bad] = self.D_MAX
        np.clip(self.profile, 0.0, self.D_MAX, out=self.profile)

        v, omega, reason = self.navigate()
        self.publish_cmd(v, omega)
        self.get_logger().info(
            f'v={v:.2f} ω={omega:+.2f} | {reason}',
            throttle_duration_sec=0.5
        )

    # ── M1: Best sector ──────────────────────────────────────────────────

    def _best_sector(self) -> tuple[int, float]:
        """
        Scan only the forward reachable arc. Score = min_profile(θ ± SECTOR_HALF)
        + FORWARD_BIAS·max(0, cosθ). Directions that turn into a tight
        side/corner are capped by that body-side clearance.
        Returns (best_dir_deg, sector_clearance_m).
        """
        p    = self.profile
        W    = self.SECTOR_HALF
        fb   = self.FORWARD_BIAS
        left_guard, right_guard = self._turn_clearances()
        best_dir, best_score, best_clr = 0, -1.0, 0.0
        for c in range(-self.SEARCH_DEG, self.SEARCH_DEG + 1):
            θ   = c % 360
            lo  = (θ - W) % 360
            hi  = (θ + W) % 360
            clr = float(p[lo:hi + 1].min() if lo <= hi
                        else min(p[lo:].min(), p[:hi + 1].min()))
            if c > 5:
                clr = min(clr, left_guard)
            elif c < -5:
                clr = min(clr, right_guard)
            score = clr + fb * math.cos(math.radians(c))
            if score > best_score:
                best_score, best_dir, best_clr = score, θ, clr
        return best_dir, best_clr

    def _lidar_front_clearance(self) -> float:
        W  = self.FRONT_HALF
        p  = self.profile
        return min(float(p[:W + 1].min()), float(p[-W:].min()))

    def _front_clearance(self) -> float:
        lidar_front = self._lidar_front_clearance()
        tof_front = min(self.tof_fl, self.tof_fr)
        if self._front_tof_pair_close():
            return min(lidar_front, tof_front)
        return lidar_front

    def _front_tof_pair_close(self) -> bool:
        return max(self.tof_fl, self.tof_fr) <= self.TOF_PAIR_STOP

    def _side_clearances(self) -> tuple[float, float]:
        p = self.profile
        left = min(float(p[45:121].min()), self.tof_fl)
        right = min(float(p[240:316].min()), self.tof_fr)
        return left, right

    def _corner_clearances(self) -> tuple[float, float]:
        p = self.profile
        left = float(p[20:66].min())
        right = float(p[294:341].min())
        return left, right

    def _turn_clearances(self) -> tuple[float, float]:
        left_side, right_side = self._side_clearances()
        left_corner, right_corner = self._corner_clearances()
        return min(left_side, left_corner), min(right_side, right_corner)

    def _front_tof_hit_side(self, threshold: float) -> str | None:
        left_hit = self.tof_fl < threshold
        right_hit = self.tof_fr < threshold
        if left_hit and not right_hit:
            return 'FL'
        if right_hit and not left_hit:
            return 'FR'
        if left_hit and right_hit:
            return 'FL' if self.tof_fl <= self.tof_fr else 'FR'
        return None

    def _limit_cmd(self, v: float, omega: float) -> tuple[float, float]:
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9
        dt = max(0.02, min(0.5, dt))

        if v < self.last_v:
            v_limited = v
        else:
            v_limited = min(v, self.last_v + self.ACCEL_LIMIT * dt)

        delta_w = self.OMEGA_ACCEL * dt
        omega_limited = float(np.clip(
            omega,
            self.last_omega - delta_w,
            self.last_omega + delta_w
        ))

        self.last_v = v_limited
        self.last_omega = omega_limited
        self.last_cmd_time = now
        return v_limited, omega_limited

    def _recovery_cmd(self, side: str, reason: str) -> tuple[float, float, str]:
        rear_clear = min(self.tof_rl, self.tof_rr)
        omega = -self.OMEGA_ESCAPE if side == 'FL' else self.OMEGA_ESCAPE
        v = self.V_ESCAPE if rear_clear >= self.REAR_CLEAR else 0.0
        self.last_v = v
        self.last_omega = omega
        self.last_cmd_time = self.get_clock().now()
        return v, omega, (
            f'{reason} {side} ToF:{self.tof_fl:.2f}/{self.tof_fr:.2f} '
            f'rear:{rear_clear:.2f}'
        )

    def _low_obstacle_cmd(self, lidar_front: float) -> tuple[float, float, str] | None:
        now_ns = self.get_clock().now().nanoseconds
        active = self.low_obstacle_side is not None

        if active:
            if self.low_obstacle_side == 'FL':
                still_close = self.tof_fl < self.ESCAPE_EXIT
            elif self.low_obstacle_side == 'FR':
                still_close = self.tof_fr < self.ESCAPE_EXIT
            else:
                still_close = min(self.tof_fl, self.tof_fr) < self.ESCAPE_EXIT

            if not still_close and now_ns >= self.low_obstacle_until_ns:
                self.low_obstacle_side = None
                return None

            side = self.low_obstacle_side
        else:
            if lidar_front < self.DVEL_SAFE:
                return None

            side = self._front_tof_hit_side(self.ESCAPE_ENTER)
            if side is None:
                return None

            self.low_obstacle_side = side
            self.low_obstacle_until_ns = now_ns + int(self.ESCAPE_MIN_TIME * 1e9)

        if side not in ('FL', 'FR'):
            self.low_obstacle_side = None
            return None

        return self._recovery_cmd(side, f'low obstacle lidarF={lidar_front:.2f}')

    def _escape_cmd(self) -> tuple[float, float, str] | None:
        now_ns = self.get_clock().now().nanoseconds
        active = self.escape_side is not None

        if active:
            if self.escape_side == 'FL':
                still_close = self.tof_fl < self.ESCAPE_EXIT
            elif self.escape_side == 'FR':
                still_close = self.tof_fr < self.ESCAPE_EXIT
            else:
                still_close = min(self.tof_fl, self.tof_fr) < self.ESCAPE_EXIT

            if not still_close and now_ns >= self.escape_until_ns:
                self.escape_side = None
                return None

            side = self.escape_side
        else:
            side = self._front_tof_hit_side(self.ESCAPE_ENTER)
            if side is None:
                return None

            self.escape_side = side
            self.escape_until_ns = now_ns + int(self.ESCAPE_MIN_TIME * 1e9)

        if side not in ('FL', 'FR'):
            self.escape_side = None
            return None

        return self._recovery_cmd(side, 'escape')

    # ── Main logic ───────────────────────────────────────────────────────

    def navigate(self) -> tuple[float, float, str]:
        lidar_front = self._lidar_front_clearance()

        low_obstacle = self._low_obstacle_cmd(lidar_front)
        if low_obstacle is not None:
            return low_obstacle

        escape = self._escape_cmd()
        if escape is not None:
            return escape

        best_dir, best_clr = self._best_sector()

        c_deg     = best_dir if best_dir <= 180 else best_dir - 360
        theta_gap = math.radians(c_deg)

        front_d = self._front_clearance()
        front_src = 'tof-pair' if self._front_tof_pair_close() and front_d < lidar_front else 'lidar'
        left_d, right_d = self._side_clearances()
        left_corner, right_corner = self._corner_clearances()
        left_guard, right_guard = min(left_d, left_corner), min(right_d, right_corner)

        if front_d <= self.FRONT_STOP:
            self.last_v = 0.0
            self.last_omega = 0.0
            self.last_cmd_time = self.get_clock().now()
            return 0.0, 0.0, (
                f'front stop d={front_d:.2f}m src={front_src} '
                f'lidarF={lidar_front:.2f} ToF:{self.tof_fl:.2f}/{self.tof_fr:.2f}'
            )

        nearest_d  = max(front_d, 0.01)
        k          = self.GAP_WEIGHT / nearest_d
        theta_final = k * theta_gap / (k + 1.0)

        wall_bias = 0.0
        if left_guard < self.SIDE_CLEAR:
            wall_bias -= (self.SIDE_CLEAR - left_guard) / self.SIDE_CLEAR
        if right_guard < self.SIDE_CLEAR:
            wall_bias += (self.SIDE_CLEAR - right_guard) / self.SIDE_CLEAR
        theta_final += math.radians(20.0) * wall_bias
        theta_final = float(np.clip(theta_final, -self.MAX_STEER, self.MAX_STEER))

        side_d = min(left_d, right_d)
        corner_d = min(left_corner, right_corner)
        reason_base = (
            f'best={c_deg:+.0f}° clr={best_clr:.2f}m '
            f'F:{front_d:.2f}({front_src}) L:{left_d:.2f} R:{right_d:.2f} '
            f'C:{corner_d:.2f} '
            f'ToF:{self.tof_fl:.2f}/{self.tof_fr:.2f} '
            f'θ={math.degrees(theta_final):+.1f}°'
        )

        sg_x = math.cos(theta_final)
        sg_y = math.sin(theta_final)
        v, omega = self.arc_cmd(sg_x, sg_y, front_d, side_d, corner_d)
        v, omega = self._limit_cmd(v, omega)
        return v, omega, reason_base

    # ── Motion command ────────────────────────────────────────────────────

    def arc_cmd(self, sg_x: float, sg_y: float, front_d: float, side_d: float, corner_d: float) -> tuple[float, float]:
        front_scale = (front_d - self.FRONT_STOP) / (self.DVEL_SAFE - self.FRONT_STOP)
        side_scale = 0.45 + 0.55 * min(1.0, side_d / self.SIDE_CLEAR)
        corner_scale = min(1.0, corner_d / self.SIDE_CLEAR)
        scale = max(0.0, min(1.0, front_scale, side_scale, corner_scale))
        vlim  = scale * self.V_MAX

        if abs(sg_y) < 1e-4:
            return float(np.clip(vlim, 0.0, self.V_MAX)), 0.0

        r       = (sg_x * sg_x + sg_y * sg_y) / (2.0 * sg_y)
        heading = math.atan(1.0 / r)
        v       = float(np.clip(vlim * math.cos(heading), 0.0, self.V_MAX))
        omega   = float(np.clip(vlim * math.sin(heading), -self.OMEGA_MAX, self.OMEGA_MAX))
        return v, omega

    # ── Publisher ─────────────────────────────────────────────────────────

    def publish_cmd(self, v: float, omega: float):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x  = float(v)
        msg.twist.angular.z = float(omega)
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DecideNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
