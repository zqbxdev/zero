# Battery telemetry reported by the hardware controller.
# 硬件控制器上报的电池状态。
builtin_interfaces/Time stamp

# Battery voltage [V].
# 电池电压 [V]。
float32 voltage_v

# Battery current [A]; positive means discharge.
# 电池电流 [A]，正值表示放电。
float32 current_a

# Estimated state of charge [%], range 0 to 100.
# 估算剩余电量 [%]，范围 0 到 100。
float32 state_of_charge_percent

# True when voltage is below the safe threshold.
# 电压低于安全阈值时为 true。
bool low_voltage

# True when the battery monitor reports a fault.
# 电池监测报告故障时为 true。
bool fault

# Human-readable fault detail; empty when fault is false.
# 可读故障信息；无故障时为空。
string fault_message
