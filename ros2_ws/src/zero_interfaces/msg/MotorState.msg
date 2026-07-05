# STM32 电机控制器反馈。 / Feedback from the STM32 motor controller for both propulsion motors.
builtin_interfaces/Time stamp

# 左电机目标输出轴转速 [rpm]。 / Left motor target output-shaft speed [rpm].
float32 left_target_rpm

# 右电机目标输出轴转速 [rpm]。 / Right motor target output-shaft speed [rpm].
float32 right_target_rpm

# 左电机实测输出轴转速 [rpm]。 / Left motor measured output-shaft speed [rpm].
float32 left_actual_rpm

# 右电机实测输出轴转速 [rpm]。 / Right motor measured output-shaft speed [rpm].
float32 right_actual_rpm

# 左霍尔编码器累计计数 [ticks]。 / Left Hall encoder accumulated count [ticks].
int64 left_encoder_count

# 右霍尔编码器累计计数 [ticks]。 / Right Hall encoder accumulated count [ticks].
int64 right_encoder_count

# 左电机带符号 PWM 占空比 [%]，范围 -100 到 100。 / Left signed PWM duty cycle [%], range -100 to 100.
float32 left_pwm_duty

# 右电机带符号 PWM 占空比 [%]，范围 -100 到 100。 / Right signed PWM duty cycle [%], range -100 to 100.
float32 right_pwm_duty

# 左电机输出使能时为 true。 / True when left motor output is enabled.
bool left_enabled

# 右电机输出使能时为 true。 / True when right motor output is enabled.
bool right_enabled

# 电机控制器报告故障时为 true。 / True when the motor controller reports a fault.
bool fault

# 可读故障信息；无故障时为空。 / Human-readable fault detail; empty when fault is false.
string fault_message
