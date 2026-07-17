# zero_mapping

Source-only mapping resource package for the Zero USV simulation stack.

This package owns the Humble `slam_toolbox` asynchronous mapping configuration,
the mapping RViz view, and the policy for keeping generated map outputs outside
the source tree. Runtime launch orchestration is owned by `zero_bringup`.

## Scope

- Provides `config/slam_toolbox.yaml` for online asynchronous mapping.
- Provides `rviz/mapping.rviz` for map, scan, TF, and robot-model inspection.
- Documents that generated occupancy-map files must be saved to an external
  prefix, not committed under `maps/`.
- Does not own AMCL, Nav2, map-server runtime configuration, path planning, or
  Gazebo process orchestration.

## Runtime Note

Build and source the ROS 2 workspace before using this package through bringup:

```bash
cd /workspace/ros2_ws
colcon build --symlink-install --packages-select zero_mapping zero_bringup
source install/setup.bash
ros2 launch zero_bringup gazebo_mapping.launch.py
```
