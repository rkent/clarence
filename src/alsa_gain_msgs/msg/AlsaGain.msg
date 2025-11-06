# Represents the gain of a particular Alsa audio control
string control    # the name given to the volume control
string device     # the alsa device name that contains the control
# percent, decibels, and muted are per-channel values
int32[] percent   # gain expressed as percent of full scale [0 - 100]
float32[] decibels  # gain expressed in decibels compared to nominal value
bool[] muted     # is the control muted?
