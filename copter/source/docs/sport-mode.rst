.. _sport-mode:

==========
Sport Mode
==========

Sport Mode is also known as "rate controlled stabilize" plus Altitude
Hold.

Overview
========

-  It was designed to be useful for flying FPV and filming `dolly shots <https://en.wikipedia.org/wiki/Dolly_shot>`__ or fly bys because
   you can set the vehicle at a particular angle and it will maintain
   that angle.
-  The pilot's roll, pitch and yaw sticks control the rate of rotation
   of the vehicle so when the sticks are released the vehicle will
   remain in it's current attitude.
-  The vehicle will not lean more than 45 degrees (this angle is
   adjustable with the ANGLE_MAX parameter)
-  The altitude is maintained with the altitude hold controller so the
   vehicle will attempt to hold it's current altitude when the sticks
   are placed with 10% of mid-throttle. It will climb or descend at up
   to 2.5m/s (this speed is adjustable with the PILOT_VELZ_MAX
   parameter)
