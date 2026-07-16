from __future__ import annotations

from typing import Final

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


# Transient-local depth-one QoS latches the latest safety command/status.
# 瞬态本地深度 1 QoS 会锁存最新安全命令或状态。
TRANSIENT_LOCAL_DEPTH_ONE_QOS: Final = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
# Latch the last e-stop command for guard nodes that join after startup.
# 为启动后加入的安全门节点锁存最新急停命令。
E_STOP_QOS: Final = TRANSIENT_LOCAL_DEPTH_ONE_QOS
# Latch the terminal initializer result for launch/orchestration observers.
# 为启动和编排观察者锁存初始化器终态结果。
INITIALIZER_STATUS_QOS: Final = TRANSIENT_LOCAL_DEPTH_ONE_QOS
