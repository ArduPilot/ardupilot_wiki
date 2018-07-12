.. _common-leds-pixhawk:

==============
LEDs (Pixhawk)
==============

This topic explains how to interpret the colours and flash sequence of
the main LED. Some of the LED patterns have associated sound/tone
patterns as listed in :ref:`Sounds (Pixhawk) <common-sounds-pixhawkpx4>`.

Video Overview
==============

..  youtube:: j-CMLrAwlco
    :width: 100%

LED Meanings
============

**Flashing red and blue**: Initializing gyroscopes. Hold the vehicle
still and level while it initializes the sensors.

**Flashing blue**: Disarmed, no GPS lock found. Autopilot, loiter and
return-to-launch modes require GPS lock.

**Solid blue**: Armed with no GPS lock

**Flashing green**: Disarmed (ready to arm), GPS lock acquired. Quick
double tone when disarming from the armed state.

**Fast Flashing green**: Same as above but GPS is using SBAS (so should
have better position estimate).

**Solid green - with single long tone at time of arming:** Armed, GPS
lock acquired. Ready to fly!

**Double flashing yellow:** Failing pre-arm checks (system refuses to
arm).

**Single Flashing yellow:** Radio failsafe activated

**Flashing yellow - with quick beeping tone**: Battery failsafe
activated

**Flashing yellow and blue - with high-high-high-low tone sequence
(dah-dah-dah-doh):** GPS glitch or GPS failsafe activated

**Flashing red and yellow - with rising tone:** EKF or Inertial Nav
failure

**Flashing purple and yellow**: Barometer glitch

**Solid Red**: Error

**Solid Red with `SOS tone sequence <http://firmware.ardupilot.org/downloads/wiki/pixhawk_sound_files/NoSDCard_short.wav>`__**:
SD Card missing (or other SD error like bad format etc.)
