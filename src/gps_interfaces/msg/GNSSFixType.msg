uint8 STATUS_NO_FIX = 0
uint8 STATUS_DEAD_RECKONING_ONLY = 1
uint8 STATUS_2D_FIX = 2 
uint8 STATUS_3D_FIX = 3
uint8 STATUS_GNSS_RECKONING_COMBINED = 4
uint8 STATUS_TIME_ONLY_FIX = 5

# STATUS_NO_FIX (0) = No Fix: No valid position fix. Receiver is searching for satellites or signal is too low.

# STATUS_DEAD_RECKONING_ONLY (1) = Dead Reckoning Only: Position based only on onboard sensors, not satellite data. 
# Used when satellites aren't visible. Accuracy may drift over time.

# STATUS_2D_FIX (2) = 2D Fix: Valid position for latitude and longitude but altitude is not accurate. Limited satellite visibility.

# STATUS_3D_FIX (3): Full position fix with latitude, longitude, and altitude. Standard, reliable GPS fix with sufficient satellites.

# STATUS_GNSS_RECKONING_COMBINED (4) = GNSS + Dead Reckoning Combined: Combines satellite and dead reckoning data to enhance position accuracy, 
#especially in challenging environments.

# STATUS_TIME_ONLY_FIX (5) = Time Only Fix: Only time data is available, not position. Useful for timing applications without location data.

uint8 status