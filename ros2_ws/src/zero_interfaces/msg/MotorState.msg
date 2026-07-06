# Feedback from the STM32 motor controller for both propulsion motors.
# STM32 电机控制器反馈。
builtin_interfaces/Time stamp

# Left motor target output-shaft speed [rpm].
# 左电机目标输出轴转速 [rpm]。
float32 left_target_rpm

# Right motor target output-shaft speed [rpm].
# 右电机目标输出轴转速 [rpm]。
float32 right_target_rpm

# Left motor measured output-shaft speed [rpm].
# 左电机实测输出轴转速 [rpm]。
float32 left_actual_rpm

# Right motor measured output-shaft speed [rpm].
# 右电机实测输出轴转速 [rpm]。
float32 right_actual_rpm

# Left Hall encoder accumulated count [ticks].
# 左霍尔编码器累计计数 [ticks]。
int64 left_encoder_count

# Right Hall encoder accumulated count [ticks].
# 右霍尔编码器累计计数 [ticks]。
int64 right_encoder_count

# Left signed PWM duty cycle [%], range -100 to 100.
# 左电机带符号 PWM 占空比 [%]，范围 -100 到 100。
float32 left_pwm_duty

# Right signed PWM duty cycle [%], range -100 to 100.
# 右电机带符号 PWM 占空比 [%]，范围 -100 到 100。
float32 right_pwm_duty

# True when left motor output is enabled.
# 左电机输出使能时为 true。
bool left_enabled

# True when right motor output is enabled.
# 右电机输出使能时为 true。
bool right_enabled

# True when the motor controller reports a fault.
# 电机控制器报告故障时为 true。
bool fault

# Human-readable fault detail; empty when fault is false.
# 可读故障信息；无故障时为空。
string fault_message
