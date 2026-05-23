# Reactive Obstacle Avoidance

This codebase implements a ROS 2 reactive obstacle avoidance pipeline for a mobile robot. The main controller uses a Follow-the-Gap (FTG) style model over a fused 360-degree obstacle profile, then sends commands through a safety gate before publishing `/cmd_vel`.

## How to Run

SSH into the Husarion robot, then build and launch the obstacle avoidance system with:

```bash
cd ~/ros2_ws && \
colcon build --packages-select rosbot_obstacle_avoidance && \
source ~/ros2_ws/install/setup.bash && \
ros2 launch rosbot_obstacle_avoidance ablation_full.launch.py
```

This launch configuration activates the full sensor stack, including:
* LiDAR
* Depth Camera
* ToF sensors

## Sensor It Use

The system uses four sensor/data sources:

| Source | ROS topic | Used by | Purpose |
| --- | --- | --- | --- |
| 2D LiDAR | `/scan_filtered` (`sensor_msgs/LaserScan`) | `lidar_reader.py` | Builds a 360-bin distance profile, one bin per degree. |
| Four ToF range sensors | `/range/fl`, `/range/fr`, `/range/rl`, `/range/rr` (`sensor_msgs/LaserScan`) | `tof_reader.py` | Detects close obstacles near front-left, front-right, rear-left, and rear-right corners. |
| OAK-D Pro depth camera | `/oak/stereo/image_raw` (`sensor_msgs/Image`) and `/oak/stereo/camera_info` (`sensor_msgs/CameraInfo`) | `depth_reader.py` | Virtual scan over the forward arc from a horizontal band of depth pixels. Catches forward-arc obstacles the LiDAR plane and ToF corners both miss. |
| Odometry | `/odometry/filtered` (`nav_msgs/Odometry`) | `odometry_reader.py` | Publishes pose and velocity for logging/analysis. It is not used by the reactive controller. |

Sensor orientation used by the obstacle profile:

| Profile index | Direction |
| --- | --- |
| `0` | Forward |
| `90` | Left |
| `180` | Rear |
| `270` or `-90` | Right |

The ToF array is published as:

```text
/tof_readings = [front_left, front_right, rear_left, rear_right]
```

Invalid ToF readings are treated as `0.9 m`, which means clear at the sensor maximum range.

## Sensor Coverage

Each sensor is intentionally responsible for a different slice of the robot's surroundings. None of them overlap completely:

| Sensor | Horizontal coverage | Height covered | Distance range | Catches |
| --- | --- | --- | --- | --- |
| ToF FL / FR | front corners (~25 deg each) | floor-level, sensor mounted on chassis | 0 to 0.9 m | very close obstacles right at the robot's front corners, including floor-level cables and small items |
| ToF RL / RR | rear corners (~25 deg each) | floor-level, sensor mounted on chassis | 0 to 0.9 m | obstacles behind the robot during reverse/recovery |
| 2D LiDAR | full 360 deg | thin slice at ~0.20 m above floor | 0.15 to 12.0 m | walls, chair legs, anything at hub height all around the robot |
| Depth camera | forward ~66 deg (HFOV) | band ~0.12 to 0.32 m above floor | 0.20 to 3.0 m | obstacles in the forward arc that sit just below or just above the LiDAR plane (boxes, low shelves at typical distances) |

The depth camera does **not** attempt drop-off detection, ground-plane fitting, or 3D point classification. It is a pure virtual scan — the simplest, most robust use of depth data for this purpose.

## Process

The active runtime pipeline is:

1. `lidar_reader.py` subscribes to `/scan_filtered`.
2. It filters invalid LiDAR rays and rejects readings outside `0.15 m` to `12.0 m`.
3. It groups valid rays into 360 one-degree bins and publishes the median distance per bin on `/lidar_profile`.
4. `tof_reader.py` subscribes to the four ToF topics and publishes `/tof_readings` at 10 Hz.
5. `depth_reader.py` subscribes to `/oak/stereo/image_raw` and `/oak/stereo/camera_info`. For each depth frame it slices a horizontal band of pixel rows around the optical center, finds the closest valid depth in each column, converts column index to bearing angle using the camera intrinsics, and publishes the result as a 360-bin profile on `/camera_profile`.
6. `build_profile_node.py` fuses `/lidar_profile`, `/camera_profile`, and `/tof_readings` into `/profile`. Fusion uses a "min wins, only lower a bin" rule:
   - LiDAR baseline.
   - Camera lowers any bin, including the central front zone, because the camera is the only sensor that can see low or overhanging obstacles directly ahead.
   - ToFs lower their projected bins. The FL and FR ToFs are excluded from the central front zone (`FRONT_PROTECT_DEG`) so the camera owns that region.
7. `decide_node.py` runs the Follow-the-Gap reactive planner on `/profile`, checks ToF readings for low/corner obstacles, and publishes `/cmd_brain`.
8. `gate_node.py` is the only node that publishes `/cmd_vel`. It clamps command limits, stops stale commands, and blocks unsafe forward or reverse motion using ToF emergency thresholds.

Supporting nodes:

| File | Role |
| --- | --- |
| `odometry_reader.py` | Converts odometry into `/robot_pose` and `/robot_velocity`. |
| `data_logger.py` | Logs state, distances, pose, velocity, and commands to CSV under `~/ros2_ws/logs`. Some subscribed topic names appear to match an older interface, so check them before using this logger with the current active pipeline. |

## Why The Depth Camera Is A Virtual Scan And Not 3D Reasoning

The depth_reader takes a thin band of pixels around the optical center and treats each column as one ray of a virtual laser scan. It does not project pixels into 3D, does not consult any TF transform, and does not classify pixels by height.

This is the same algorithm used by the standard ROS `depthimage_to_laserscan` package. The reason it works robustly:

- **Floor is excluded by image geometry, not by a height threshold.** With the band sitting at `cy +/- 20` rows and the OAK-D Pro intrinsics (`fy` around `618`, camera at ~0.22 m above floor), the floor first projects into the band only at distance `fy * h_cam / 20` which is around 6.8 m, well past the 3 m profile cap. The band literally cannot see the floor inside the configured range.
- **No calibration of camera mounting is needed.** Only the camera's intrinsics matter, and those come from `/oak/stereo/camera_info` automatically.
- **One real knob: band width.** Wider band catches shorter obstacles at closer range but starts including the floor at long range. Narrower band excludes the floor entirely but misses some short obstacles up close. The default of `+/- 20` rows is a safe starting point.

What the camera does not do, on purpose:

- No drop-off detection — unreliable from a single forward camera without ground-plane fitting.
- No overhang detection above ~0.32 m — outside the robot's collision envelope in practice.
- No semantic classification — the planner does not need to know what an obstacle is, only that it is there.

## The Reactive Model: Follow The Gap

The controller in `decide_node.py` is a reactive planner. It does not use a map, a global path, TF, or a goal pose. Each decision is made from the latest fused obstacle profile and ToF readings.

The FTG behavior works like this:

1. Search only the reachable forward corridor from `-60 deg` to `+60 deg`.
2. For each candidate direction, inspect a sector of `theta +/- 20 deg`.
3. Score the direction by the minimum clearance in that sector plus a forward bias:

```text
score(theta) = min_clearance(theta +/- SECTOR_HALF) + FORWARD_BIAS * cos(theta)
```

4. Reduce the score for turns toward close side or corner obstacles.
5. Pick the best-scoring direction as the gap direction.
6. Blend the gap direction back toward straight ahead based on front clearance:

```text
k = GAP_WEIGHT / front_distance
theta_final = (k * theta_gap) / (k + 1)
```

7. Add wall bias if a left or right side/corner is too close.
8. Clamp steering to `+/-45 deg`.
9. Convert the selected steering direction into a curved arc command `(linear velocity, angular velocity)`.
10. Reduce speed when front, side, or corner clearance is low.
11. Enter escape/recovery mode if front ToF sensors detect a close low obstacle.

Important safety behavior:

- If front clearance is below `FRONT_STOP`, the planner commands zero velocity.
- If one front ToF corner is very close, the robot turns away from that side.
- If rear ToF clearance is sufficient, recovery may include a small reverse velocity.
- `gate_node.py` applies the final hard stop and command limits before `/cmd_vel`.

## Diagram Of The Logic

```mermaid
flowchart TD
    A["/scan_filtered<br/>2D LiDAR"] --> B["lidar_reader.py<br/>filter rays + median 360 bins"]
    B --> C["/lidar_profile"]

    D["/range/fl, /range/fr,<br/>/range/rl, /range/rr<br/>ToF sensors"] --> E["tof_reader.py<br/>validate + publish ToF array"]
    E --> F["/tof_readings"]

    S["/oak/stereo/image_raw<br/>/oak/stereo/camera_info<br/>OAK-D Pro depth"] --> T["depth_reader.py<br/>pixel band -&gt; virtual scan"]
    T --> U["/camera_profile"]

    C --> G["build_profile_node.py<br/>fuse LiDAR + camera + ToF"]
    U --> G
    F --> G
    G --> H["/profile<br/>fused 360-degree obstacle profile"]

    H --> I["decide_node.py<br/>Follow-the-Gap reactive planner"]
    F --> I

    I --> J{"Low obstacle<br/>or escape needed?"}
    J -- "yes" --> K["Recovery command<br/>turn away, reverse if rear clear"]
    J -- "no" --> L["Search -45 deg to +45 deg<br/>score sectors by clearance"]
    L --> M["Pick best gap<br/>blend toward forward"]
    M --> N["Apply wall bias<br/>clamp steering"]
    N --> O["Arc command<br/>scale speed by clearance"]
    K --> P["/cmd_brain"]
    O --> P

    P --> Q["gate_node.py<br/>watchdog, emergency stops, command limits"]
    F --> Q
    Q --> R["/cmd_vel"]
```

## Hyperparameters

### LiDAR Profile

| Parameter | Value | Meaning |
| --- | ---: | --- |
| `lidar_reader.MIN_RANGE` | `0.15 m` | Ignore readings closer than this. |
| `lidar_reader.MAX_RANGE` | `12.0 m` | Ignore readings farther than this and use as empty-bin value. |
| LiDAR bins | `360` | One distance bin per degree. |

### ToF Fusion

| Parameter | Value | Meaning |
| --- | ---: | --- |
| `tof_reader.MAX_RANGE` | `0.9 m` | ToF maximum range; also used as clear value. |
| `build_profile_node.TOF_HALF_FOV` | `12.5 deg` | Half field-of-view projected into profile bins. |
| `build_profile_node.FRONT_PROTECT_DEG` | `10 deg` | Front zone protected from direct FL/FR ToF lowering. The camera owns this zone. |
| FL center | `+12.5 deg` | Front-left ToF projection center. |
| FR center | `-12.5 deg` | Front-right ToF projection center. |
| RL center | `+167.5 deg` | Rear-left ToF projection center. |
| RR center | `-167.5 deg` | Rear-right ToF projection center. |

### Depth Camera

| Parameter | Value | Meaning |
| --- | ---: | --- |
| `depth_reader.BAND_ROWS_ABOVE` | `20` | Rows above the optical center included in the scan band. |
| `depth_reader.BAND_ROWS_BELOW` | `20` | Rows below the optical center included in the scan band. |
| `depth_reader.MIN_DEPTH` | `0.20 m` | OAK-D Pro reliable minimum depth; closer readings are discarded. |
| `depth_reader.MAX_DEPTH` | `3.0 m` | Cap on camera-derived obstacle distance; matches planner `D_MAX`. |
| `depth_reader.N_BINS` | `360` | One bin per degree, same convention as the LiDAR profile. |
| `depth_reader.MAX_RANGE` | `3.0 m` | Sentinel value for bins where the band sees no obstacle. |
| `build_profile_node.CAMERA_MAX` | `3.0 m` | Match value used during fusion; must match `depth_reader.MAX_RANGE`. |

### Follow-the-Gap Planner

| Parameter | Value | Meaning |
| --- | ---: | --- |
| `R` | `0.167 m` | Robot half-width. |
| `D_MAX` | `3.0 m` | Planner profile max/no-obstacle sentinel. |
| `SECTOR_HALF` | `20 deg` | Half-width of each scored clearance sector. |
| `FORWARD_BIAS` | `0.25 m` | Bonus for forward-facing candidate directions. |
| `GAP_WEIGHT` | `80.0` | Weight used when blending gap direction with forward. |
| `SEARCH_DEG` | `60 deg` | Candidate search corridor from `-60 deg` to `+60 deg`. |
| `FRONT_HALF` | `10 deg` | Front stop clearance sector. |
| `SIDE_CLEAR` | `0.40 m` | Clearance where side/corner wall bias begins. |
| `MAX_STEER` | `45 deg` | Maximum final steering angle. |

### Speed And Recovery

| Parameter | Value | Meaning |
| --- | ---: | --- |
| `FRONT_STOP` | `0.22 m` | Planner stop distance. |
| `TOF_PAIR_STOP` | `0.28 m` | Front ToF pair threshold used when both front ToFs agree. |
| `DVEL_SAFE` | `0.75 m` | Distance where speed reduction begins. |
| `V_MAX` | `0.18 m/s` | Planner maximum linear velocity. |
| `OMEGA_MAX` | `0.6 rad/s` | Planner maximum angular velocity. |
| `ACCEL_LIMIT` | `0.08 m/s^2` | Linear acceleration limit. |
| `OMEGA_ACCEL` | `0.5 rad/s^2` | Angular acceleration limit. |
| `ESCAPE_ENTER` | `0.28 m` | ToF threshold to enter escape mode. |
| `ESCAPE_EXIT` | `0.36 m` | ToF threshold to leave escape mode. |
| `ESCAPE_MIN_TIME` | `1.0 s` | Minimum time to stay in escape mode. |
| `REAR_CLEAR` | `0.30 m` | Rear clearance needed for reverse recovery. |
| `V_ESCAPE` | `-0.05 m/s` | Reverse speed during recovery. |
| `OMEGA_ESCAPE` | `0.35 rad/s` | Turn rate during recovery. |

### Safety Gate

| Parameter | Value | Meaning |
| --- | ---: | --- |
| `EMERGENCY_DIST` | `0.25 m` | Front ToF distance that latches a front emergency. |
| `EMERGENCY_CLEAR` | `0.32 m` | Front clearance needed to release the latch. |
| `REAR_EMERGENCY_DIST` | `0.18 m` | Rear ToF distance that latches a rear emergency. |
| `REAR_EMERGENCY_CLEAR` | `0.28 m` | Rear clearance needed to release the latch. |
| `WATCHDOG_TIMEOUT` | `0.5 s` | Stop if `/cmd_brain` is stale. |
| `V_LIMIT` | `0.16 m/s` | Final velocity clamp before `/cmd_vel`. |
| `OMEGA_LIMIT` | `0.6 rad/s` | Final angular velocity clamp before `/cmd_vel`. |