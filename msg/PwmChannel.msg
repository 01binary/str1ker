# modes
uint8 MODE_ANALOG=0   # Use analogWrite to set value
uint8 MODE_DIGITAL=1  # Use digitalWrite to set value

uint8 channel         # Channel that maps to a pin
uint8 mode            # Channel mode, analog or digital
uint16 value          # Value to set: duty cycle if analog, 1 or 0 if digital
uint8 duration        # Duration in microseconds after which the value should be inverted (0 if not used)
