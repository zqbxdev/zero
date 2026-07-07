# Zero ROS2 包技术备忘

这份备忘只记录当前源码里已经存在的 ROS2 包和接口。判断依据是 `ros2_ws/src/**/package.xml`，再用同包内的 `CMakeLists.txt`、launch、URDF、msg、srv 文件核对技术细节。现有 `AGENTS.md` 和路线图文档只作为背景参考，若和源码不一致，以源码为准。

## 包选择规则

纳入清单的规则：必须在 `ros2_ws/src/` 下存在真实 `package.xml`。按当前源码，已有下面五个包：

| 包名 | 类型 | 当前状态 | 主要依据 |
| --- | --- | --- | --- |
| `zero_description` | `ament_cmake` 资源和显示包 | 已有 URDF、STL、display launch 和安装规则 | `package.xml`、`CMakeLists.txt`、`launch/display.launch.py`、`urdf/robot.urdf` |
| `zero_interfaces` | `ament_cmake` 接口包 | 已有 4 个 msg 和 1 个 srv，由 rosidl 生成 | `package.xml`、`CMakeLists.txt`、`msg/*.msg`、`srv/SetControlMode.srv` |
| `zero_hardware` | `ament_python` fake hardware 包 | 已有 fake motor controller，验证接口闭环，不接真实硬件 | `package.xml`、`setup.py`、`zero_hardware/fake_motor_controller.py`、`zero_hardware/fake_motor_model.py` |
| `zero_sim` | `ament_python` 最小二维仿真包 | 已有纯二维运动模型和 `/odom`、`odom -> zero_base_link` 发布节点 | `package.xml`、`setup.py`、`zero_sim/simple_usv_model.py`、`zero_sim/simple_usv_simulator.py`、`test/test_simple_usv_model.py` |
| `zero_bringup` | `ament_python` 启动编排包 | 已有 fake 和 sim 一键启动入口，sim 入口拉起 fake hardware、二维仿真、URDF TF 和 RViz | `package.xml`、`setup.py`、`launch/bringup_fake.launch.py`、`launch/bringup_sim.launch.py` |

未在 `ros2_ws/src/` 下出现 `package.xml` 的名字，例如 `zero_control`、`zero_localization`、`zero_navigation`、`zero_gazebo`、`zero_safety`，都只能视为未实现的未来方向，不能写成当前功能。

## zero_description

### 关键文件

| 文件或目录 | 作用 |
| --- | --- |
| `ros2_ws/src/zero_description/package.xml` | 声明包名、版本、许可证、运行依赖和 `ament_cmake` 构建类型。运行依赖包含 `joint_state_publisher_gui`、`robot_state_publisher`、`rviz2`、`tf2_ros`。 |
| `ros2_ws/src/zero_description/CMakeLists.txt` | 安装运行资源目录：`launch`、`meshes`、`rviz`、`urdf`。 |
| `ros2_ws/src/zero_description/launch/display.launch.py` | 模型显示入口。读取安装后的 URDF，启动 `robot_state_publisher`、临时静态 `map -> zero_base_link` TF 和 `rviz2`。 |
| `ros2_ws/src/zero_description/urdf/robot.urdf` | 当前机器人模型源。机器人名是 `zero_usv`。 |
| `ros2_ws/src/zero_description/meshes/` | URDF 引用的 STL 网格资源，路径使用 `package://zero_description/meshes/...`。 |
| `ros2_ws/src/zero_description/rviz/` | 被安装的 RViz 资源目录。当前未见固定 RViz 配置内容。 |
| `ros2_ws/src/zero_description/cad_export/solidworks/` | CAD 导出来源记录，不由 CMake 安装，不是运行时输入。 |

### 设计和技术点

`zero_description` 只负责机器人描述和人工显示检查。它不包含硬件通信、控制、定位、导航、Gazebo 仿真或安全逻辑。

`display.launch.py` 的行为很明确：先通过 `get_package_share_directory('zero_description')` 定位安装后的 share 目录，再读取 `urdf/robot.urdf`，把完整文本作为 `robot_description` 传给 `robot_state_publisher`。同一个 launch 还启动 `tf2_ros/static_transform_publisher`，发布临时 `map -> zero_base_link` 静态 TF，最后启动 `rviz2`。这个 `map` 固定关系是显示用入口，不代表定位链路已经实现。

URDF 中机器人名是 `zero_usv`，主要 link 包括 `zero_base_link`、`lidar_link`、`left_motor_link`、`right_motor_link`。固定关节把雷达和左右电机 link 接到 `zero_base_link`。网格路径都使用 `package://zero_description/meshes/...`，这和安装后的 ROS2 包解析方式匹配。

当前 collision 几何复用 visual STL，适合先做显示和基本结构检查，不应直接理解为已经有仿真优化碰撞模型。`right_motor_link` 的质量和惯性全部为 0，源码注释也提示仍需实物核对。`cad_export/solidworks` 只保留 SolidWorks 导出来源和记录，不参与运行。

### 构建和使用命令

```bash
cd /workspace/ros2_ws
colcon build --symlink-install --packages-select zero_description
source install/setup.bash
ros2 launch zero_description display.launch.py
```

### Gotchas

1. `display.launch.py` 读取的是安装后的 `urdf/robot.urdf`，不是源码树文件路径。修改 URDF 后需要重新构建或确认安装资源已更新。
2. `CMakeLists.txt` 只安装 `launch`、`meshes`、`rviz`、`urdf`。新运行资源如果放到别处，必须同步安装规则。
3. `package.xml` 声明了 `joint_state_publisher_gui`，但当前 display launch 没有启动它。
4. `rviz/` 是安装目录，但当前没有固定 RViz 工程配置可依赖。
5. `map -> zero_base_link` 是临时显示 TF。未来定位包若出现，应由定位链路正式管理 map、odom、base 等坐标关系。

## zero_interfaces

### 关键文件

| 文件 | 作用 |
| --- | --- |
| `ros2_ws/src/zero_interfaces/package.xml` | 声明接口包依赖。包含 `rosidl_default_generators`、`builtin_interfaces`、`rosidl_default_runtime`，并加入 `rosidl_interface_packages` 组。 |
| `ros2_ws/src/zero_interfaces/CMakeLists.txt` | 调用 `rosidl_generate_interfaces(${PROJECT_NAME} ...)` 生成接口，依赖 `builtin_interfaces`，并 `ament_export_dependencies(rosidl_default_runtime)`。 |
| `ros2_ws/src/zero_interfaces/msg/MotorCommand.msg` | 左右推进电机目标转速命令。 |
| `ros2_ws/src/zero_interfaces/msg/MotorState.msg` | STM32 电机控制器反馈结构。 |
| `ros2_ws/src/zero_interfaces/msg/BatteryState.msg` | 硬件控制器上报的电池状态。 |
| `ros2_ws/src/zero_interfaces/msg/UsvStatus.msg` | 无人船高层运行模式和故障状态。 |
| `ros2_ws/src/zero_interfaces/srv/SetControlMode.srv` | 请求切换高层控制模式。 |

### 设计和技术点

`zero_interfaces` 是当前公共接口包。它只定义消息和服务，不写业务节点、硬件通信、控制算法或 launch。接口注释采用中英双语，并把关键单位写在字段注释里，例如 rpm、ticks、百分比、V、A。

构建配置已经进入接口包形态：`package.xml` 使用 `rosidl_default_generators` 作为构建工具依赖，依赖 `builtin_interfaces`，运行时导出 `rosidl_default_runtime`，并声明 `member_of_group` 为 `rosidl_interface_packages`。`CMakeLists.txt` 通过 `rosidl_generate_interfaces` 生成 `MotorCommand`、`MotorState`、`BatteryState`、`UsvStatus` 和 `SetControlMode`，依赖 `builtin_interfaces`。

### 消息和服务字段摘要

| 接口 | 字段和常量 |
| --- | --- |
| `MotorCommand.msg` | `stamp`，`left_target_rpm`，`right_target_rpm`。左右目标输出轴转速单位为 rpm。 |
| `MotorState.msg` | `stamp`，左右目标 rpm，左右实测 rpm，左右霍尔编码器累计计数 `left_encoder_count` 和 `right_encoder_count`，左右带符号 PWM 占空比 `left_pwm_duty` 和 `right_pwm_duty`，左右输出使能 `left_enabled` 和 `right_enabled`，`fault`，`fault_message`。PWM 注释标明范围为 -100 到 100。 |
| `BatteryState.msg` | `stamp`，`voltage_v`，`current_a`，`state_of_charge_percent`，`low_voltage`，`fault`，`fault_message`。电流正值表示放电，剩余电量范围是 0 到 100。 |
| `UsvStatus.msg` | 常量 `MODE_UNKNOWN=0`、`MODE_STOP=1`、`MODE_MANUAL=2`、`MODE_AUTO=3`、`MODE_FAULT=4`。字段为 `stamp`、`mode`、`fault`、`fault_code`、`fault_message`。`fault_code` 为 0 表示无故障。 |
| `SetControlMode.srv` | 请求常量 `MODE_STOP=1`、`MODE_MANUAL=2`、`MODE_AUTO=3`，请求字段 `mode`。响应字段为 `accepted`、`current_mode`、`message`。 |

### 构建和使用命令

```bash
cd /workspace/ros2_ws
colcon build --symlink-install --packages-select zero_interfaces
source install/setup.bash
ros2 interface show zero_interfaces/msg/MotorCommand
ros2 interface show zero_interfaces/msg/MotorState
ros2 interface show zero_interfaces/msg/BatteryState
ros2 interface show zero_interfaces/msg/UsvStatus
ros2 interface show zero_interfaces/srv/SetControlMode
```

### Gotchas

1. 这个包现在有真实 `msg/` 和 `srv/` 文件。旧文档或 AGENTS 中“没有接口文件”的说法已经过时。
2. 接口字段一旦被其他包消费，字段名、字段类型和常量值就会形成契约，修改前需要同步消费者。
3. 这些接口描述的是 ROS2 层契约，不等于已经存在硬件节点、控制节点、fake hardware、串口协议或集成测试。
4. `MotorState.msg` 注释提到 STM32 电机控制器反馈，这是接口语义，不代表当前仓库已有 STM32 通信实现。

## zero_hardware

### 关键文件

| 文件 | 作用 |
| --- | --- |
| `ros2_ws/src/zero_hardware/package.xml` | 声明 fake hardware 包依赖。包含 `ament_python`、`rclpy` 和 `zero_interfaces`。 |
| `ros2_ws/src/zero_hardware/setup.py` | 安装 Python 包，并注册 `fake_motor_controller` console script。 |
| `ros2_ws/src/zero_hardware/setup.cfg` | 把 ROS2 可执行脚本安装到 `lib/zero_hardware`。 |
| `ros2_ws/src/zero_hardware/zero_hardware/fake_motor_controller.py` | ROS2 节点入口，连接 `/zero/*` topic 和 service。 |
| `ros2_ws/src/zero_hardware/zero_hardware/fake_motor_model.py` | 不依赖 ROS2 的 fake 电机状态模型，供节点和测试复用。 |
| `ros2_ws/src/zero_hardware/test/test_fake_motor_model.py` | 纯模型测试，覆盖 STOP、MANUAL、AUTO、无效模式和编码器计数。 |

### 设计和技术点

`zero_hardware` 当前只实现 fake hardware 最小闭环。节点名是 `fake_motor_controller`，订阅 `/zero/motor_command`，周期发布 `/zero/motor_state`、`/zero/battery_state` 和 `/zero/status`，并提供 `/zero/set_control_mode` 服务。

`fake_motor_model.py` 把可测试的状态推进逻辑从 ROS2 callback 中分离出来。默认模式是 STOP；MANUAL 和 AUTO 会接受目标转速命令，实际转速用简单限幅步进跟随目标；STOP 会忽略新命令，并把目标转速和实际转速逐步拉回 0。当前 fake 电池和状态消息固定发布无故障值，只用于接口链路验证。

这个包不连接 STM32，不实现串口协议、checksum、PID、导航、规划、Gazebo 或真实硬件控制。`MotorState` 中编码器计数和 PWM duty 是模拟值，只用于验证上层 topic/service 契约。

### 构建和使用命令

```bash
cd /workspace/ros2_ws
colcon build --symlink-install --packages-select zero_interfaces zero_hardware
source install/setup.bash
ros2 run zero_hardware fake_motor_controller
```

另开终端验证：

```bash
source /workspace/ros2_ws/install/setup.bash
ros2 topic list
ros2 service call /zero/set_control_mode zero_interfaces/srv/SetControlMode "{mode: 2}"
ros2 topic pub --once /zero/motor_command zero_interfaces/msg/MotorCommand "{left_target_rpm: 120.0, right_target_rpm: 120.0}"
ros2 topic echo /zero/motor_state
ros2 service call /zero/set_control_mode zero_interfaces/srv/SetControlMode "{mode: 1}"
```

### Gotchas

1. 这个包目前是 fake 闭环，不代表已有真实硬件桥接或 STM32 通信。
2. `/zero/set_control_mode` 只接受 STOP、MANUAL 和 AUTO；无效模式会返回 `accepted=false`，并保持当前模式。
3. STOP 模式会逐步回零，不是瞬间清零；验收时应观察连续状态反馈。
4. 运行 ROS2 graph 验证前需要先构建并 source `install/setup.bash`，否则生成的 `zero_interfaces` Python 类型不可用。

## zero_sim

### 关键文件

| 文件 | 作用 |
| --- | --- |
| `ros2_ws/src/zero_sim/package.xml` | 声明最小二维仿真包依赖。包含 `ament_python`、`rclpy`、`zero_interfaces`、`builtin_interfaces`、`nav_msgs`、`geometry_msgs` 和 `tf2_ros`。 |
| `ros2_ws/src/zero_sim/setup.py` | 安装 Python 包，并注册 `simple_usv_simulator` console script。 |
| `ros2_ws/src/zero_sim/setup.cfg` | 把 ROS2 可执行脚本安装到 `lib/zero_sim`。 |
| `ros2_ws/src/zero_sim/zero_sim/simple_usv_model.py` | 不依赖 ROS2 的二维平面运动模型，按左右实际 rpm 积分 `x`、`y`、`yaw`，并返回线速度和角速度。 |
| `ros2_ws/src/zero_sim/zero_sim/simple_usv_simulator.py` | ROS2 节点入口，订阅 `/zero/motor_state`，发布 `/odom` 并广播 `odom -> zero_base_link` TF。 |
| `ros2_ws/src/zero_sim/test/test_simple_usv_model.py` | 纯模型 pytest 覆盖前进、后退、转向、零 rpm 和非零航向积分。 |

### 设计和技术点

`zero_sim` 当前只实现最小二维运动仿真。节点名是 `simple_usv_simulator`，订阅 `zero_interfaces/msg/MotorState` 的 `/zero/motor_state`，只使用 `left_actual_rpm` 和 `right_actual_rpm`，不绕过 fake hardware 直接读取命令。

纯模型 `simple_usv_model.py` 没有 ROS2 import。默认参数是 `rpm_to_linear_velocity=0.001`、`rpm_difference_to_angular_velocity=0.002`，线速度取左右实际 rpm 平均值乘线速度系数，角速度取 `(right_actual_rpm - left_actual_rpm)` 乘差速角速度系数。yaw 为 0 时左右同正 rpm 会沿 +x 前进；z、roll、pitch 保持 0，orientation 由平面 yaw 转四元数。

`simple_usv_simulator.py` 启动后即使用零 rpm 周期发布 `/odom` 和 `odom -> zero_base_link` TF，因此还没收到第一条 `/zero/motor_state` 时也能看到静止里程计。它只发布局部 `odom` 链路，不发布 `map -> odom` 或 `map -> zero_base_link`。

### 构建和使用命令

```bash
cd /workspace/ros2_ws
colcon build --symlink-install --packages-select zero_interfaces zero_hardware zero_sim
source install/setup.bash
ros2 run zero_sim simple_usv_simulator
```

配合 fake hardware 验证：

```bash
source /workspace/ros2_ws/install/setup.bash
ros2 run zero_hardware fake_motor_controller
ros2 topic echo /odom
ros2 run tf2_ros tf2_echo odom zero_base_link
```

### Gotchas

1. 这个包不是 Gazebo、水动力、定位、导航、SLAM、路径规划或真实硬件桥接。
2. `/odom` 来自 `/zero/motor_state` 中的实际 rpm；如果 fake hardware 处于 STOP 且实际 rpm 为 0，位姿会保持静止。
3. 运行仿真时不能同时启动 `zero_description/launch/display.launch.py` 中的静态 `map -> zero_base_link`，否则 `zero_base_link` 会有两个父坐标系。

## zero_bringup

### 关键文件

| 文件 | 作用 |
| --- | --- |
| `ros2_ws/src/zero_bringup/package.xml` | 声明启动编排包依赖。包含 `ament_python`、`launch`、`launch_ros`、`zero_hardware`、`zero_sim`、`zero_description`、`robot_state_publisher` 和 `rviz2`。 |
| `ros2_ws/src/zero_bringup/setup.py` | 安装 Python 包、包资源标记、`package.xml` 和 fake/sim launch 文件。 |
| `ros2_ws/src/zero_bringup/setup.cfg` | 保持 `ament_python` 脚本安装路径约定。当前包不注册 console script。 |
| `ros2_ws/src/zero_bringup/launch/bringup_fake.launch.py` | fake 系统一键启动入口，启动 `zero_hardware` 的 `fake_motor_controller`。 |
| `ros2_ws/src/zero_bringup/launch/bringup_sim.launch.py` | sim 系统一键启动入口，读取安装后的 `zero_description/urdf/robot.urdf`，启动 `robot_state_publisher`、`fake_motor_controller`、`simple_usv_simulator` 和 `rviz2`。 |

### 设计和技术点

`zero_bringup` 当前只负责启动编排。`bringup_fake.launch.py` 只拉起已有 `zero_hardware` 节点，不重新定义 topic、service、消息、服务、电机模型、硬件通信、控制算法或仿真模型。

fake 系统入口命令是 `ros2 launch zero_bringup bringup_fake.launch.py`。启动后实际 topic 和 service 仍由 `zero_hardware` 的 `fake_motor_controller` 提供：订阅 `/zero/motor_command`，发布 `/zero/motor_state`、`/zero/battery_state`、`/zero/status`，并提供 `/zero/set_control_mode`。

sim 系统入口命令是 `ros2 launch zero_bringup bringup_sim.launch.py`。这个入口不会包含 `zero_description/launch/display.launch.py`，也不会启动 `static_transform_publisher`；URDF 固定关节由 `robot_state_publisher` 发布，船体运动 TF 由 `zero_sim` 发布为 `odom -> zero_base_link`。

### 构建和使用命令

```bash
cd /workspace/ros2_ws
colcon build --symlink-install --packages-select zero_interfaces zero_description zero_hardware zero_sim zero_bringup
source install/setup.bash
ros2 launch zero_bringup bringup_fake.launch.py
ros2 launch zero_bringup bringup_sim.launch.py
```

另开终端验证：

```bash
source /workspace/ros2_ws/install/setup.bash
ros2 topic list
ros2 service list
ros2 service call /zero/set_control_mode zero_interfaces/srv/SetControlMode "{mode: 2}"
ros2 topic pub --once /zero/motor_command zero_interfaces/msg/MotorCommand "{left_target_rpm: 120.0, right_target_rpm: 120.0}"
ros2 topic echo /zero/motor_state
ros2 service call /zero/set_control_mode zero_interfaces/srv/SetControlMode "{mode: 1}"
```

### Gotchas

1. 这个包是系统入口整理，不是新的业务节点包；不要在这里放电机模型、硬件通信、控制算法或仿真模型。
2. launch 文件不 remap `/zero/*` 名称，命名契约继续由 `zero_hardware` 和 `zero_interfaces` 维护。
3. `bringup_sim.launch.py` 读取安装后的 URDF；修改 `zero_description` 资源后需要重新构建或确认安装资源已更新。
4. 修改 launch 文件后需要重新构建或确认安装资源已更新，再运行 `ros2 launch`。

## 维护检查清单

1. 先用 `ros2_ws/src/**/package.xml` 重新生成包清单，再更新本备忘。
2. 新增包只有在包目录和 `package.xml` 真实存在后，才移动到当前包清单。
3. 更新 `zero_description` 时，同步检查 `CMakeLists.txt` 安装目录、`display.launch.py` 读取路径、URDF link 名和 mesh 路径。
4. 更新 URDF 惯性或碰撞模型时，特别复核 `right_motor_link` 的零质量和零惯性，以及 collision 是否仍复用 visual STL。
5. 更新 `zero_interfaces` 时，同步检查 `package.xml`、`CMakeLists.txt`、每个 msg/srv 字段、单位注释和常量值。
6. 更新 `zero_sim` 时，同步检查纯模型测试、`/odom`、`odom -> zero_base_link` TF 和 `bringup_sim.launch.py` 是否仍避免静态 `map -> zero_base_link`。
7. 不从路线图反推当前实现。硬件、控制、定位、导航、Gazebo、安全和集成测试，只有源码路径和包清单出现后才能写成已实现。
8. 维护命令只写当前包可以直接对应的构建、launch 或 `ros2 interface show` 命令，不写未来包的假入口。
