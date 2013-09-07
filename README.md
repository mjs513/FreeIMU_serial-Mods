FreeIMU_serial-Mods
===================

Modifications to to FreeIMU_serial.ino from Fabio Versano's FreeIMU library to dump all calibarated data from 
the IMU for display and further processing. Two additional options were added to the code when quering the
sketch:
1. "a" - no Kalman filtering applied to the quaternions
2. "z" - Kalman filtering applied to the quaternions

Both options dump all data to the serial port.
