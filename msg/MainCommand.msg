# Normalized velocity commands in the range [-1, 1].
float32 headPanVelocity
float32 headTiltVelocity
float32 torsoPanVelocity
float32 torsoTiltVelocity

# Mouth servo target in degrees, clamped by the firmware to [0, 180].
int16 mouthPosition
