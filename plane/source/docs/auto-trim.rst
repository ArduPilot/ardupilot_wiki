.. _auto-trim:

==============
Automatic Trim
==============

ArduPlane provides a means to trim the pitch and roll control surfaces to correct for small trim errors, such that when flying in MANUAL mode, the plane will be in the same flying attitude as when flying in throttle controlled modes (AUTO, CRUISE,etc.) or FBWA.


SERVO AUTO TRIM
---------------

By setting :ref:`SERVO_AUTO_TRIM<SERVO_AUTO_TRIM>` to 1, ArduPlane will constantly monitor how much it needs to correct the flying trim to maintain level AHRS pitch and roll while in the aforementioned modes. Every 10 seconds, it will store this trim into the appropriate ``SERVOx_TRIM`` values for outputs assigned to control roll and pitch (ie Aileron/Elevator, Elevons, etc.). This includes flaperon and differential spoiler configurations. After a minute or so of flight, the trims will be adjusted such that minor trim errors will be canceled out when switching back to MANUAL and ACRO modes.

The amount that can be trimmed is limited to about 25% of the throw, so very radical out-of-trim conditions are not correctable. Also, it will not be updated unless there is no pilot input, the plane is being controlled by the autopilot to be in generally level flight, and the plane is flying above 8m/s ground speed.

.. note:: This does not substitute for having an incorrect CG. It will compensate for minor trim issues with small CG variations, but a badly located CG will still make the Plane uncontrollable and crashes inevitable.

TRIM AUTO
---------

This function was used to allow easy trimming of ArduPlane in the earlier versions of the firmware, and is still included for backward compatibility, although not recommended for use since :ref:`SERVO_AUTO_TRIM<SERVO_AUTO_TRIM>`  is now available. It was intended to be used and then de-activated, since incorrect trim values could be captured if stick inputs were being accidentally applied when the switch out of MANUAL mode occurred.

If :ref:`TRIM_AUTO<TRIM_AUTO>` is enabled, every time MANUAL mode is exited, the output values of the roll/pitch assigned outputs was captured as their input and output trim values. So the user could use the transmitter trim tabs to correct the aircraft roll and pitch trim, and it would be saved upon exit. The trim tabs would then be left in their non-neutral positions, yet not interfere with the other flight modes as unwanted pilot inputs. This feature may be deprecated in the future. Please use the safer :ref:`SERVO_AUTO_TRIM<SERVO_AUTO_TRIM>` feature.



