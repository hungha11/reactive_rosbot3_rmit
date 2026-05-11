import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import TwistStamped
import csv
import os
from datetime import datetime


class DataLogger(Node):
    """
    Logs all system data to CSV for report analysis.

    Subscribes to:
        /robot_state        String
        /obstacle_state     String
        /lidar_state        String
        /tof_state          String
        /obstacle_distances Float32MultiArray [front,fl,fr,left,right,rear]
        /tof_distances      Float32MultiArray [fl,fr,rl,rr]
        /robot_pose         Float32MultiArray [x,y,yaw]
        /robot_velocity     Float32MultiArray [vx,wz]
        /raw_cmd_vel        TwistStamped
        /cmd_vel            TwistStamped
        /safety_status      String

    Output:
        ~/ros2_ws/logs/obstacle_avoidance_<timestamp>.csv
    """

    CSV_HEADERS = [
        'timestamp',
        'robot_state',
        'obstacle_state',
        'lidar_state',
        'tof_state',
        # LiDAR distances
        'lidar_front',
        'lidar_fl',
        'lidar_fr',
        'lidar_left',
        'lidar_right',
        'lidar_rear',
        # ToF distances
        'tof_fl',
        'tof_fr',
        'tof_rl',
        'tof_rr',
        # Pose
        'pos_x',
        'pos_y',
        'yaw',
        # Velocity
        'vx',
        'wz',
        # Commands
        'raw_cmd_linear',
        'raw_cmd_angular',
        'cmd_linear',
        'cmd_angular',
        # Events
        'event',
    ]

    def __init__(self):
        super().__init__('data_logger')

        # Internal state — latest values from each topic
        self.robot_state    = 'UNKNOWN'
        self.obstacle_state = 'UNKNOWN'
        self.lidar_state    = 'UNKNOWN'
        self.tof_state      = 'UNKNOWN'
        self.lidar_dist     = [18.0] * 6
        self.tof_dist       = [0.9]  * 4
        self.pose           = [0.0]  * 3
        self.velocity       = [0.0]  * 2
        self.raw_cmd        = [0.0]  * 2
        self.cmd            = [0.0]  * 2

        # Event tracking
        self.last_robot_state    = 'UNKNOWN'
        self.last_obstacle_state = 'UNKNOWN'
        self.pending_event       = ''

        # Set up log directory and file
        log_dir = os.path.expanduser('~/ros2_ws/logs')
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_path = os.path.join(log_dir, f'obstacle_avoidance_{timestamp}.csv')

        self.csv_file   = open(self.log_path, 'w', newline='')
        self.csv_writer = csv.DictWriter(
            self.csv_file, fieldnames=self.CSV_HEADERS)
        self.csv_writer.writeheader()
        self.csv_file.flush()

        # Subscribers
        self.create_subscription(
            String, '/robot_state',
            self.robot_state_cb, 10)

        self.create_subscription(
            String, '/obstacle_state',
            self.obstacle_state_cb, 10)

        self.create_subscription(
            String, '/lidar_state',
            self.lidar_state_cb, 10)

        self.create_subscription(
            String, '/tof_state',
            self.tof_state_cb, 10)

        self.create_subscription(
            Float32MultiArray, '/obstacle_distances',
            self.lidar_dist_cb, 10)

        self.create_subscription(
            Float32MultiArray, '/tof_distances',
            self.tof_dist_cb, 10)

        self.create_subscription(
            Float32MultiArray, '/robot_pose',
            self.pose_cb, 10)

        self.create_subscription(
            Float32MultiArray, '/robot_velocity',
            self.velocity_cb, 10)

        self.create_subscription(
            TwistStamped, '/raw_cmd_vel',
            self.raw_cmd_cb, 10)

        self.create_subscription(
            TwistStamped, '/cmd_vel',
            self.cmd_cb, 10)

        # Log at 5Hz — enough for report, not too much data
        self.create_timer(0.2, self.log_row)

        self.get_logger().info(f'DataLogger started → {self.log_path}')

    # ── Callbacks ────────────────────────────────────────────────────────

    def robot_state_cb(self, msg: String):
        if msg.data != self.last_robot_state:
            self.pending_event = f'STATE_CHANGE:{self.last_robot_state}->{msg.data}'
            self.last_robot_state = msg.data
        self.robot_state = msg.data

    def obstacle_state_cb(self, msg: String):
        if msg.data != self.last_obstacle_state:
            self.last_obstacle_state = msg.data
        self.obstacle_state = msg.data

    def lidar_state_cb(self, msg: String):
        self.lidar_state = msg.data

    def tof_state_cb(self, msg: String):
        self.tof_state = msg.data

    def lidar_dist_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 6:
            self.lidar_dist = list(msg.data)

    def tof_dist_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            self.tof_dist = list(msg.data)

    def pose_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self.pose = list(msg.data)

    def velocity_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            self.velocity = list(msg.data)

    def raw_cmd_cb(self, msg: TwistStamped):
        self.raw_cmd = [
            msg.twist.linear.x,
            msg.twist.angular.z
        ]

    def cmd_cb(self, msg: TwistStamped):
        self.cmd = [
            msg.twist.linear.x,
            msg.twist.angular.z
        ]

    # ── Logging ──────────────────────────────────────────────────────────

    def log_row(self):
        """Write one row to CSV at 5Hz."""
        row = {
            'timestamp':       self.get_clock().now().nanoseconds / 1e9,
            'robot_state':     self.robot_state,
            'obstacle_state':  self.obstacle_state,
            'lidar_state':     self.lidar_state,
            'tof_state':       self.tof_state,
            # LiDAR distances
            'lidar_front':     round(self.lidar_dist[0], 3),
            'lidar_fl':        round(self.lidar_dist[1], 3),
            'lidar_fr':        round(self.lidar_dist[2], 3),
            'lidar_left':      round(self.lidar_dist[3], 3),
            'lidar_right':     round(self.lidar_dist[4], 3),
            'lidar_rear':      round(self.lidar_dist[5], 3),
            # ToF distances
            'tof_fl':          round(self.tof_dist[0], 3),
            'tof_fr':          round(self.tof_dist[1], 3),
            'tof_rl':          round(self.tof_dist[2], 3),
            'tof_rr':          round(self.tof_dist[3], 3),
            # Pose
            'pos_x':           round(self.pose[0], 3),
            'pos_y':           round(self.pose[1], 3),
            'yaw':             round(self.pose[2], 3),
            # Velocity
            'vx':              round(self.velocity[0], 3),
            'wz':              round(self.velocity[1], 3),
            # Commands
            'raw_cmd_linear':  round(self.raw_cmd[0], 3),
            'raw_cmd_angular': round(self.raw_cmd[1], 3),
            'cmd_linear':      round(self.cmd[0], 3),
            'cmd_angular':     round(self.cmd[1], 3),
            # Events
            'event':           self.pending_event,
        }

        self.csv_writer.writerow(row)
        self.csv_file.flush()

        # Clear event after logging
        if self.pending_event:
            self.get_logger().info(
                f'Logged event: {self.pending_event}',
                throttle_duration_sec=1.0
            )
            self.pending_event = ''

    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info(f'DataLogger closed → {self.log_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()