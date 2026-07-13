# zero_gazebo

Minimal Gazebo Fortress sensor smoke package for the Zero USV phase-4 observation layer.

## File Map

```text
zero_gazebo/
├── launch/gazebo_sensors.launch.py        # Starts Gazebo, robot_state_publisher, and ros_gz_bridge.
├── config/ros_gz_bridge.yaml              # Owns Gazebo -> ROS topic bridge mappings.
├── models/zero_usv/model.sdf              # Owns the Gazebo simplified model and sensors.
├── worlds/zero_sensor_smoke.sdf           # Owns the smoke-test world and scan target.
└── urdf/zero_usv_sensors.urdf.xacro       # Owns ROS robot_description and fixed sensor TF.
```

This package intentionally uses both SDF and xacro:

- SDF is used for Gazebo because the sensors and simulated geometry run there.
- xacro is used for ROS because `robot_state_publisher` needs a maintainable robot description for TF.
- The two files must keep the same frame poses for `zero_base_link`, `lidar_link`, `laser_frame`, and `imu_link`.

## Scope

- Provides a simplified primitive `model://zero_usv`, not the SolidWorks mesh model.
- Publishes Gazebo `/scan`, `/imu`, and `/clock` through `ros_gz_bridge` into ROS 2.
- Publishes only the fixed TF relationships `zero_base_link -> lidar_link`, `lidar_link -> laser_frame`, and `zero_base_link -> imu_link` through `robot_state_publisher`.
- Makes the simulated lidar approximate the real YDLIDAR X2 used by this project: `laser_frame`, 360 degree scan, 0.10-8.0 m range, 6 Hz scan rate, and about 500 samples per scan.
- Adds one static box target in the world so `/scan` can show finite range readings during smoke tests.
- Does not add localization, SLAM, Nav2, autonomy, motor control, hydrodynamics, or safety-chain behavior.

## Where To Change Things

- Change ROS bridge topics or message types in `config/ros_gz_bridge.yaml`.
- Change X2-like simulated lidar parameters in `models/zero_usv/model.sdf` under the `zero_lidar` sensor.
- Change fixed ROS TF poses in `urdf/zero_usv_sensors.urdf.xacro`, then mirror the same pose in `models/zero_usv/model.sdf`.
- Change smoke-test obstacles or world setup in `worlds/zero_sensor_smoke.sdf`.
- Keep `launch/gazebo_sensors.launch.py` small; it should wire resources together, not hide model or bridge parameters.

## Build

```bash
cd /workspace/ros2_ws
colcon build --symlink-install --packages-select zero_gazebo
source install/setup.bash
```

## Launch

```bash
ros2 launch zero_gazebo gazebo_sensors.launch.py
```

## Verify

```bash
ros2 topic echo /scan --once
ros2 topic hz /scan
ros2 topic echo /imu --once
ros2 topic hz /imu
ros2 topic echo /clock --once
ros2 run tf2_ros tf2_echo zero_base_link lidar_link
ros2 run tf2_ros tf2_echo lidar_link laser_frame
ros2 run tf2_ros tf2_echo zero_base_link imu_link
```

The simulated lidar keeps the Zero project mount frame `lidar_link`, while `/scan` uses `laser_frame` to match the physical `ydlidar_ros2` driver default. `laser_frame` is fixed at the same pose as `lidar_link` for this phase-4 smoke model.

Optional xacro-only check:

```bash
xacro $(ros2 pkg prefix zero_gazebo)/share/zero_gazebo/urdf/zero_usv_sensors.urdf.xacro
```

Use the project's `full` Docker image or an equivalent ROS 2 Humble environment with Gazebo Fortress, `ros_gz_bridge`, and `xacro` installed.
