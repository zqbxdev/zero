# 硬件控制器上报的电池状态。 / Battery telemetry reported by the hardware controller.
builtin_interfaces/Time stamp

# 电池电压 [V]。 / Battery voltage [V].
float32 voltage_v

# 电池电流 [A]，正值表示放电。 / Battery current [A]; positive means discharge.
float32 current_a

# 估算剩余电量 [%]，范围 0 到 100。 / Estimated state of charge [%], range 0 to 100.
float32 state_of_charge_percent

# 电压低于安全阈值时为 true。 / True when voltage is below the safe threshold.
bool low_voltage

# 电池监测报告故障时为 true。 / True when the battery monitor reports a fault.
bool fault

# 可读故障信息；无故障时为空。 / Human-readable fault detail; empty when fault is false.
string fault_message
