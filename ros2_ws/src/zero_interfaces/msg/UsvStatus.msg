# 无人船高层运行模式和故障状态。 / High-level USV operating mode and fault status.
uint8 MODE_UNKNOWN=0
uint8 MODE_STOP=1
uint8 MODE_MANUAL=2
uint8 MODE_AUTO=3
uint8 MODE_FAULT=4

builtin_interfaces/Time stamp

# 当前运行模式，使用 MODE_* 常量。 / Current operating mode; use MODE_* constants.
uint8 mode

# 无人船处于或报告故障状态时为 true。 / True when the USV is in or reporting a fault condition.
bool fault

# 数值故障码，0 表示无故障。 / Numeric fault code; 0 means no fault.
uint32 fault_code

# 可读故障信息；无故障时为空。 / Human-readable fault detail; empty when fault is false.
string fault_message
