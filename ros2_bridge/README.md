# ROS 2 bridge

The excavator simulator as ROS 2 (Jazzy) topics: joint states, TF, lidar point cloud, depth image, IMU, bucket and truck loads, all in simulation time. Lever commands go back to the machine.

Nothing is installed on the host. The simulator (`phase2/ros_server.py`) runs on the host in its Python environment. The ROS 2 node runs in a Docker container. The two talk over a local TCP socket.

```
host: phase2/ros_server.py  ──TCP 7777──▶  container: excavator_bridge.py  ──DDS──▶  your ROS 2 nodes
       (Newton, Warp, GPU)   ◀── levers ──                (rclpy, Jazzy)
```

## Run

```bash
# 1. the simulator (host, from the repository root)
(cd phase2 && ../phase0/.venv-newton/bin/python ros_server.py --soil loam)
```

```bash
# 2. the bridge (once: docker build -t construction-sim-ros2 ros2_bridge)
docker run --rm -it --network host --ipc host construction-sim-ros2
```

Then use any ROS 2 Jazzy environment on the same host, for example another container: `docker run --rm -it --network host --ipc host construction-sim-ros2 bash`. Inside it:

```bash
ros2 topic list
ros2 topic echo /joint_states
ros2 topic pub -r 10 /excavator/levers std_msgs/msg/Float64MultiArray "{data: [0.0, 1.0, 0.0, 0.0]}"  # raise the boom
ros2 topic pub --once /excavator/autopilot std_msgs/msg/Bool "{data: true}"                       # built-in operator
```

`--ipc host` matters: without it, Fast DDS tries shared memory between containers and no data arrives.

Options:
- **Simulator:** `ros_server.py` takes `--soil` (dry-sand, moist-sand, loam, clay), `--autopilot`, `--fast` (as fast as possible instead of real time), `--host` and `--port`.
- **Bridge:** it takes `--ros-args -p host:=… -p port:=…`.

## Topics

Stamps are in simulation time, so run your nodes with `use_sim_time:=true`.

| Topic | Type | Rate | Content |
|---|---|---|---|
| `/clock` | rosgraph_msgs/Clock | 30 Hz | simulation time |
| `/joint_states` | sensor_msgs/JointState | 30 Hz | `swing_joint`, `boom_joint`, `arm_joint`, `bucket_joint`. Position, velocity, and the hydraulic pressure on each joint as effort (Pa, + head side) |
| `/tf` | tf2_msgs/TFMessage | 30 Hz | `world → base_link`, `world → body_link` |
| `/tf_static` | tf2_msgs/TFMessage | latched | `body_link → lidar_link`, `camera_link`, `camera_optical_frame`, `imu_link` |
| `/lidar/points` | sensor_msgs/PointCloud2 | 10 Hz | 32 × 1024 spinning lidar on the cab: fields x, y, z (float32) and label (uint8) |
| `/camera/depth/image_raw` | sensor_msgs/Image | 10 Hz | 320 × 240 32FC1 depth along the optical axis (m), 0 = no return |
| `/camera/depth/camera_info` | sensor_msgs/CameraInfo | 10 Hz | pinhole intrinsics, no distortion |
| `/camera/labels/image_raw` | sensor_msgs/Image | 10 Hz | mono8 semantic label per pixel |
| `/imu/data` | sensor_msgs/Imu | 60 Hz | angular velocity, specific force; no orientation |
| `/excavator/levers_state` | std_msgs/Float64MultiArray | 30 Hz | levers in use |
| `/excavator/bucket_tip` | geometry_msgs/PointStamped | 30 Hz | cutting edge, `world` |
| `/excavator/soil_force` | geometry_msgs/Vector3Stamped | 30 Hz | soil force on the bucket (N), `world` |
| `/excavator/bucket_load`, `/excavator/truck_load` | std_msgs/Float64 | 30 Hz | kg |
| `/excavator/stage` | std_msgs/String | 30 Hz | `manual`, or the autopilot's stage |

Subscribed:
- **`/excavator/levers`** (std_msgs/Float64MultiArray): [swing (+ left), boom (+ raise), stick (+ out), bucket (+ dump)], each in [−1, 1]. Publish at 10 Hz or more: the levers return to neutral 0.5 s after the last message (a dead-man switch).
- **`/excavator/autopilot`** (std_msgs/Bool): the built-in operator drives. Any lever message takes over again.
- **`/excavator/reset`** (std_msgs/Empty): restores the machine, the pit and the truck.

Semantic labels: 0 none, 1 ground, 2 soil, 3 machine, 4 truck.

## Frames and the ZX200 model

- **`world`:** the simulator's site frame (z up). The pit spans x −3…6.75 m and y −2.5…6.25 m; the machine stands at x 7.4.
- **Joint names and angles:** these are the ones in PWRI OPERA's [`zx200_ros2`](https://github.com/pwri-opera/zx200_ros2) description (Apache-2.0).
- **`base_link`:** it is published so that the URDF chain `base_link → track_link → body_link → …` reproduces the simulator's links. Run `robot_state_publisher` with that description to see the real ZX200 in RViz, posed by `/joint_states`.
- **Without the URDF:** use `world → body_link` and the sensor frames.
- **Sensor frames:**
  - `lidar_link` and `imu_link` are axis-aligned with `body_link` (x forward along the arm, y left, z up).
  - `camera_optical_frame` follows the usual ROS optical convention (z forward, x right, y down).

## Check

With the simulator and the bridge running:

```bash
docker run --rm --network host --ipc host -v $PWD/ros2_bridge:/check construction-sim-ros2 python3 /check/check_bridge.py
```

It runs four checks:
- **Rates:** reports the rate of every topic.
- **Decoding:** decodes a point cloud and a depth image.
- **Transforms:** checks the static transforms.
- **Commands:** raises the boom through `/excavator/levers`, then lets the autopilot fill the bucket.

On a GB10 it reported:
- **Rates:** 30 Hz state, 60 Hz IMU, about 9.5 Hz lidar and camera.
- **Lidar:** 14 k points per scan.
- **Commands:** the boom moved 37° in 2 s; the bucket filled to 1073 kg in 6 s on autopilot.

## Limits

- **One simulator, one machine:** one bridge per simulator.
- **Single-machine transport:** the socket is meant for one host (`127.0.0.1`). Across machines, run `ros_server.py --host 0.0.0.0`; the protocol is unauthenticated, so only on a trusted network.
- **No `ros2_control` hardware interface yet:** commands are lever positions (valve openings), as an operator gives them, not joint positions or velocities.
- **Ideal sensors:** they are ideal raycasts, with no noise, beam divergence or rolling shutter (see `phase2/sensors.py`).
