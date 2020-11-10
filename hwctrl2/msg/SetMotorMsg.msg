# id of the motor/actuator
int8 id

# setpoint of the motor/actuator system (could be RPM or position)
float32 setpoint

# desired acceleration of the motor/actuator
float32 acceleration

# ---
# status - indicates success or failure
# 0: success
# 1: no confirmation received
# 2: write error
# int8 status

# actual acceleration being used (may not be equal to requested acceleration)
# float32 actual_accel
