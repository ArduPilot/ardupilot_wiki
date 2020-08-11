===========
Calibration
===========

There are several commands to calibrate the various sensors on the APM:

accelcal
========

Initiates the accelerometer calibration routines. This is the "place the
UAV on its left side..." routine.

accelcalsimple
==============

Does a simple pre-flight accelerometer calibration

gyrocal
=======

Initiates the gyro calibration routines.

calpress
========

Initiates the pressure sensor calibration routines.

compassmot
==========

Initiates the magnetometer/motor interference calibration routines.

level
=====

Do a levelling routine on a multicopter.

rccal
=====

Perform calibration of the RC input channels.

ground
======

Performs a full ground start. This includes gyro and pressure sensor
calibration. It is automatically performed when the APM receives an
arming command.

magcal
======

Magnetometer (compass) calibration to account for any soft-iron 
sources on the UAV.

ahrstrim
========

Do a AHRS trim.

