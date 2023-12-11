.. _auto-trim:

==============
Automatic Trim
==============

ArduPlane provides a means to trim the pitch and roll control surfaces to correct for small trim errors, such that when flying in MANUAL mode, the plane will be in the same flying attitude as when flying in throttle controlled modes (AUTO, CRUISE,etc.) or FBWA.


SERVO AUTO TRIM
---------------

By setting :ref:`SERVO_AUTO_TRIM<SERVO_AUTO_TRIM>` to 1, ArduPlane will constantly monitor how much it needs to correct the flying trim (on Roll and Pitch output control surfaces) to maintain level AHRS pitch and roll while in the aforementioned modes. Every 10 seconds, it will store this trim into the appropriate ``SERVOx_TRIM`` values for outputs assigned to control roll and pitch (ie Aileron/Elevator, Elevons, etc.). This includes flaperon and differential spoiler configurations. After a minute or so of flight, the trims will be adjusted such that minor trim errors will be canceled out when switching back to MANUAL and ACRO modes.

The amount that can be trimmed is center in the servo output range and limited to about 20% of the total ``SEROVX_MAX/MIN`` range, so very radical out-of-trim conditions are not correctable. Also, it will not be updated unless there is no pilot input, the plane is being controlled by the autopilot to be in generally level flight, and the plane is flying above 8m/s ground speed.

For example if the elevator servo has the following values for min/max, 1100/1900, then the auto trim can change the trim value in the range of 1420 to 1580us. 

.. note:: This does not substitute for having an incorrect CG. It will compensate for minor trim issues with small CG variations, but a badly located CG will still make the Plane uncontrollable and crashes inevitable.



