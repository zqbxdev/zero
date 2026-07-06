# High-level USV operating mode and fault status.
# 无人船高层运行模式和故障状态。
uint8 MODE_UNKNOWN=0
uint8 MODE_STOP=1
uint8 MODE_MANUAL=2
uint8 MODE_AUTO=3
uint8 MODE_FAULT=4

builtin_interfaces/Time stamp

# Current operating mode; use MODE_* constants.
# 当前运行模式，使用 MODE_* 常量。
uint8 mode

# True when the USV is in or reporting a fault condition.
# 无人船处于或报告故障状态时为 true。
bool fault

# Numeric fault code; 0 means no fault.
# 数值故障码，0 表示无故障。
uint32 fault_code

# Human-readable fault detail; empty when fault is false.
# 可读故障信息；无故障时为空。
string fault_message
