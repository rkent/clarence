# Information per servo used for speed and neutral position
# Info is given in terms of PMW ticks, which are typically in the range of 500-2500
uint8 servo_number   # servo number (typically 0-15)
float32 speed        # max speed, ticks/second (default 1000)
int16 rest           # rest position, PMW ticks (default 1500)
int16 max            # maximum position, PWM ticks (default 2500)
int16 min            # minimum position, PMW ticks (default 500)
