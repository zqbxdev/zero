# Zero ROS2 包技术备忘

本备忘以 `ros2_ws/src/*/package.xml` 和同包源码为事实来源。当前恰好有九个
ROS 2 包，`zero_sim` 不存在。Phase 1 complete。Phases 2-5 统一为
`implementation complete, static/build verified`，runtime 统一为
`NOT RUN / UNVERIFIED`。

## 当前九包清单

| 包名 | 类型 | 当前源码职责 |
| --- | --- | --- |
| `zero_description` | `ament_cmake` | 唯一 ROS URDF、网格、display entry 和固定传感器 frame。 |
| `zero_interfaces` | `ament_cmake` | 4 个 msg 和 1 个 srv。 |
| `zero_control` | `ament_python` | `/cmd_vel` watchdog、Twist 到 raw RPM、共享正逆运动学。 |
| `zero_safety` | `ament_python` | command guard、双状态 fault/freshness、急停锁存和 initializer。 |
| `zero_hardware` | `ament_python` | fake controller、实际 RPM、仿真 encoder 和最终命令 watchdog。 |
| `zero_gazebo` | `ament_cmake` + Python | Fortress SDF/world/bridge、actual-RPM adapter、odom-to-TF relay。 |
| `zero_mapping` | `ament_cmake` | Humble slam_toolbox、mapping RViz、外部 map output policy。 |
| `zero_navigation` | `ament_cmake` | Humble AMCL、map server、NavFn、DWB、costmap、BT、RViz 资源。 |
| `zero_bringup` | `ament_python` | shared config 和三个公共 Gazebo launch composition。 |

旧 follower 已删除。`zero_navigation` 当前没有 Python runtime executable，
也不发布 `/cmd_vel`。Nav2 composition 是 navigation 模式的 `/cmd_vel` 来源。

## launch 和 composition

| 入口 | 所有内容 | 明确不拥有 |
| --- | --- | --- |
| `zero_description/display.launch.py` | 独立模型显示、display-only 静态 TF、RViz | 公共仿真 control/mapping/navigation composition |
| `zero_bringup/bringup_fake.launch.py` | isolated fake controller | 完整 control chain |
| `zero_gazebo/gazebo_sensors.launch.py` | isolated sensor world、RSP、sensor bridge | propulsion、mapping、navigation |
| `zero_bringup/control_chain.launch.py` | 一个 RSP、control、safety、fake hardware、adapter、initializer | Gazebo process、bridge、SLAM、Nav2、RViz |
| `zero_bringup/gazebo_control.launch.py` | navigation world、bridge、control chain、一个 odom relay | SLAM、AMCL、Nav2、RViz |
| `zero_bringup/gazebo_mapping.launch.py` | control entry、一个 async slam_toolbox、可选 mapping RViz | AMCL、map server、Nav2 |
| `zero_bringup/gazebo_navigation.launch.py` | control entry、一个 Nav2 bringup、可选 navigation RViz | slam_toolbox、直接 Nav2 node duplication |

`gazebo_control.launch.py` 只公开 `world` 和 `config_file`。Mapping 和 navigation
入口额外公开 `rviz`。Navigation 还要求无默认值的 `map` 参数。

## 命令链

```text
/cmd_vel
  -> zero_control/twist_to_motor_command
  -> /zero/motor_command_raw
  -> zero_safety/command_guard
  -> /zero/motor_command
  -> zero_hardware/fake_motor_controller
  -> /zero/motor_state, actual RPM
  -> zero_gazebo/gazebo_drive_adapter
  -> /zero/gazebo/cmd_vel
  -> ros_gz_bridge, ROS_TO_GZ
  -> /model/zero_usv/cmd_vel
  -> Fortress DiffDrive System
```

所有 consumer 从 `zero_bringup/config/v1_sim.yaml` 获取同一组仿真参数。
adapter 只消费 actual RPM，不消费 target RPM 或 `/cmd_vel`。它遇到 stale、fault、
非有限、越界或 clock rollback 时的源码策略是发布零 Twist，但运行行为尚未验证。

## `/odom` 和 TF 所有权

| 关系 | 当前源码唯一所有者 |
| --- | --- |
| Gazebo `/model/zero_usv/odometry` 到 ROS `/odom` | `ros_gz_bridge.yaml` 的单向 `GZ_TO_ROS` entry |
| `odom -> zero_base_link` | `zero_gazebo/odom_to_tf_relay` |
| 船体到 `lidar_link`、`laser_frame`、`imu_link` | `zero_description/robot.urdf` 加一个 RSP |
| Mapping `map -> odom` | `slam_toolbox` |
| Navigation `map -> odom` | AMCL through Nav2 composition |

DiffDrive 的私有 `/model/zero_usv/diff_drive_tf_unbridged` 不在 bridge 配置中。
adapter 和 relay 都不积分 pose，也不发布 `/odom`。Mapping 和 navigation 不能
同时拥有 `map -> odom`。

唯一 ROS robot description 路径是
`zero_description/urdf/robot.urdf`。其中 LiDAR mount 相对
`zero_base_link` 的位姿是 `-0.00015 -0.00040397 0.1277`，IMU 相对
`zero_base_link` 的位姿是 `0.15 0 0.13`。SDF 的 sensor frame 名称和位姿必须
与这份 URDF 保持一致。

## bridge 和 world

`zero_gazebo/config/ros_gz_bridge.yaml` 有五条 one-way entry：

| ROS topic | Gazebo topic | Direction |
| --- | --- | --- |
| `/scan` | `/scan` | `GZ_TO_ROS` |
| `/imu` | `/imu` | `GZ_TO_ROS` |
| `/clock` | `/clock` | `GZ_TO_ROS` |
| `/zero/gazebo/cmd_vel` | `/model/zero_usv/cmd_vel` | `ROS_TO_GZ` |
| `/odom` | `/model/zero_usv/odometry` | `GZ_TO_ROS` |

`zero_sensor_smoke.sdf` 继续保留。`zero_navigation_world.sdf` 是三个公共 Gazebo
入口默认使用的有界静态 world。模型资源目录由
`IGN_GAZEBO_RESOURCE_PATH` 和 `GZ_SIM_RESOURCE_PATH` 指向。

## Mapping 和 map policy

`zero_mapping/config/slam_toolbox.yaml` 使用 `map`、`odom`、`zero_base_link`、
`/scan`、0.05 m resolution 和 simulation time。`mapping.rviz` 固定 frame 为
`map`。

仓库没有生成的 occupancy map 或 pose graph。未来 map saver 必须写到外部：

```bash
ros2 run nav2_map_server map_saver_cli -f <external-prefix>
```

这只是未来 runtime procedure。生成的 `.yaml`、`.pgm` 和 `.png` 不属于
`zero_mapping/maps/` source。

## Navigation 和 external map

`zero_navigation/config/nav2_params.yaml` 提供 Humble contracts：

- AMCL frame 为 `map`、`odom`、`zero_base_link`。
- map server 的 `yaml_filename` 为空，由 launch 重写。
- planner ID `GridBased` 使用 `nav2_navfn_planner/NavfnPlanner`。
- controller ID `FollowPath` 使用 `dwb_core::DWBLocalPlanner`。
- global costmap 使用 static、obstacle、inflation layers。
- local costmap 使用 obstacle 和 inflation layers。
- footprint hull 是 1.20 m × 0.55 m 矩形，padding 单独配置。
- BT Navigator 使用安装的 Humble 默认 navigate-to-pose behavior tree。

`gazebo_navigation.launch.py` 要求 `map:=` 为现有小写 `.yaml` 文件的非空绝对
路径。它以 `slam=False`、`use_sim_time=True`、`autostart=True` 和
`use_composition=True` 包含一次 Nav2 bringup。

## use_sim_time 和共享配置

`v1_sim.yaml` 为以下节点提供参数：

```text
robot_state_publisher
twist_to_motor_command
command_guard
fake_motor_controller
gazebo_drive_adapter
odom_to_tf_relay
simulation_safety_initializer
```

所有 clock consumer 都设置 `use_sim_time: true`。公共值包括 300 RPM、
1.0 m/s、1.0 rad/s、0.407 m track width、0.05 m wheel radius、左右仿真
motor sign 为 `1`，以及 command/status/motor-state timeout。2048 encoder ticks
只属于 fake simulation semantics，不是真实 hardware calibration。

## 当前状态矩阵

| 范围 | 状态 |
| --- | --- |
| Phase 1 cleanup and contract alignment | complete |
| Phase 2 control and safety implementation | `implementation complete, static/build verified` |
| Phase 3 Gazebo drive, odom, TF implementation | `implementation complete, static/build verified` |
| Phase 4 mapping resources and composition | `implementation complete, static/build verified` |
| Phase 5 Nav2 resources and composition | `implementation complete, static/build verified` |
| Static/build verification for phases 2-5 | Todo 25 complete；ARM64 image、459 source pytest、九包 build、449 package-native results 和安装树已检查 |

## Runtime criteria

| Criterion | Status |
| --- | --- |
| Fortress plugin load | `NOT RUN / UNVERIFIED` |
| Motion | `NOT RUN / UNVERIFIED` |
| `/odom` | `NOT RUN / UNVERIFIED` |
| TF uniqueness | `NOT RUN / UNVERIFIED` |
| SLAM quality | `NOT RUN / UNVERIFIED` |
| Map reload | `NOT RUN / UNVERIFIED` |
| AMCL localization | `NOT RUN / UNVERIFIED` |
| NavFn planning | `NOT RUN / UNVERIFIED` |
| DWB planning | `NOT RUN / UNVERIFIED` |
| Avoidance | `NOT RUN / UNVERIFIED` |
| Goal arrival | `NOT RUN / UNVERIFIED` |
| Stop criterion | `NOT RUN / UNVERIFIED` |

## 历史验证记录，原文保留

以下内容属于早期八包基线，不代表当前 phases 2-5 验证结果：

| 范围 | 静态证据 | 当前边界 |
| --- | --- | --- |
| 三个核心纯模型文件 | 共有 34 个顶层 `def test_` 函数，分别为 fake motor 7、twist 转换 13、command guard 14，即 7 + 13 + 14。 | 本次在隔离容器中运行仓库级 Pytest，相关测试全部包含在 55/55 passed 结果中；八包 clean build 通过。 |
| `zero_navigation` 纯 Python 测试 | 四个测试模块共有 21 个 ROS-free 顶层测试函数，覆盖输入、控制、新鲜度、安全状态和参数校验。 | 21 个测试包含在本次 55/55 passed 结果中；未执行 navigation launch、ROS graph 或端到端 smoke。 |
| CMake lint 配置 | `zero_description`、`zero_interfaces`、`zero_gazebo` 等包在 `BUILD_TESTING` 下配置 `ament_lint_auto`。 | lint 配置不等于 Gazebo 或 ROS 运行时验证。 |
| 功能集成层 | 递归检查 `ros2_ws/src/` 后，没有发现 `launch_testing` 标记，也没有自动化 ROS graph、launch 或端到端测试。 | 该行属于早期基线；当前已有 navigation composition source，但 Gazebo topic、TF 和 navigation runtime 仍未执行。 |

## 维护检查

1. 从 `package.xml` 重新枚举包，不从路线图推断。
2. 从 launch、YAML、SDF、URDF、CMake 和 setup install rules 核对资源。
3. 不把 static source feasibility 写成 runtime success。
4. 不把测试 fixture、pose graph 或空目录写成生成地图。
5. 只有重跑 Todo 25 的完整非运行时门禁后，才维持 `static/build verified`。
