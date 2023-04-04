# modes
uint8 MODE_PWM               = 0 # Set duty cycle
uint8 MODE_EFFORT            = 1 # Set effort/torque/current
uint8 MODE_HOLD              = 2 # Hold position
uint8 MODE_VELOCITY          = 3 # Set velocity
uint8 MODE_POSITION          = 4 # Set position
uint8 MODE_ORIGIN            = 5 # Set origin
uint8 MODE_POSITION_VELOCITY = 6 # Set position and velocity
uint8 MODE_RAW               = 7 # Set position, velocity, torque, stiffness, and damping

uint8 id                         # Servo id
uint8 mode                       # Channel mode (see above)
int16 position                   # Position (MODE_POSITION, MODE_POSITION_VELOCITY, and MODE_RAW)
int16 velocity                   # Velocity or Duty Cycle (MODE_PWM, MODE_VELOCITY, MODE_POSITION_VELOCITY, and MODE_RAW)
uint8 effort                     # Effort or Torque (MODE_EFFORT, MODE_RAW)
uint16 kp                        # Stiffness (MODE_RAW)
uint8 kd                         # Damping (MODE_RAW)
