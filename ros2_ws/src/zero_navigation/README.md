# zero_navigation

Source-only Nav2 resource package for the Zero USV simulation stack.

This package owns the Humble Nav2 parameter file and navigation RViz view. It is
intentionally a resource package, not a custom navigation runtime package.

## Scope

- Provides `config/nav2_params.yaml` for AMCL, map server, NavFn planner, DWB
  controller, costmaps, BT navigator, and behavior server configuration.
- Provides `rviz/navigation.rviz` for map, scan, costmap, path, initial-pose,
  and goal interaction during navigation checks.
- Uses the installed Nav2 packaged default navigate-to-pose behavior tree; no
  copied BT XML belongs in this package.
- Relies on `zero_bringup` to own launch composition and the one lifecycle
  manager through `nav2_bringup`.

## Ownership Boundaries

- No custom waypoint follower is shipped here.
- No custom recovery server is shipped here.
- No package-local map output or generated map resource is shipped here.
- Runtime map selection is provided by `zero_bringup` through an external map
  path passed to Nav2.

## Runtime Note

Build and source the ROS 2 workspace before launching navigation through
bringup:

```bash
cd /workspace/ros2_ws
colcon build --symlink-install --packages-select zero_navigation zero_bringup
source install/setup.bash
ros2 launch zero_bringup gazebo_navigation.launch.py map:=/absolute/path/map.yaml
```
