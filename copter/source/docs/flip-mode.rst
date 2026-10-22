.. _flip-mode:

=========
Flip Mode
=========

Vehicle will flip on its roll or pitch axis depending upon the pilot's roll and pitch stick position in flight mode's which allow this (ACRO/ALTHOLD/STABILIZE...and,of course, FLIP mode). Vehicle will increase throttle rapidly as it starts to flip. Once the flip is completed, canceled, or times out, the original flight mode the vehicle was in will be restored. The flip will end at the entry attitude.

The vehicle will not flip again until the switch is brought low and back to high, if on an :ref:`Auxiliary Switch <common-auxiliary-functions>`, or if the mode channel switch is changed to another mode and back to FLIP.

.. warning:: Give yourself at least 10m of altitude before trying flip for the first time!

Flip Mode Controls
==================
The mode may be entered either by an :ref:`Auxiliary Switch <common-auxiliary-functions>`, or by changing flight mode to FLIP.

- The direction of the flip defaults to ROLL LEFT, but if the RC Pitch stick is moved slightly back or forward, it will flip on the pitch axis, back or forward, respectively. If the RC Pitch stick is neutral but the RC ROLL is pushed slightly right, it will flip rolling to the right.
- During the flip the throttle is managed to attempt to neither gain or lose altitude. It is only an attempt!
- You may abort the flip by moving the pitch or roll stick as if to command near full stick in that axis. The Flip will immediately halt **at whatever attitude it is currently at** and return to the previous flight mode at the pilot's throttle stick input.
- As the flip is completing, it will briefly increase throttle to try to recover any lost altitude. Again, this is an approximation. Once completed (entry attitude re-attained), the previous flight mode is returned to (if not entered by an AUX switch). It will not flip again until the Aux Switch is lowered and raised again, if used, or FLIP flight mode re-entered. If entered via a flight mode switch, you will need to change mode out of FLIP, to another mode, and then back again if another flip is desired,

Flip Rotation Rate
==================

The rate at which the vehicle rotates during the flip is set by :ref:`FLIP_RATE<FLIP_RATE>`, in deg/s. It defaults to 400 deg/s and is constrained to the range 60 to 1000 deg/s.

Lower rates give a slower, gentler flip, which is usually required on larger vehicles and on traditional helicopters, since they cannot reach the default rate.

The flip's timeout is derived from this parameter: the flip is abandoned if it has not completed within twice the time a full 360 deg rotation would take at :ref:`FLIP_RATE<FLIP_RATE>`, i.e. 720/:ref:`FLIP_RATE<FLIP_RATE>` seconds, which is 1.8s at the default rate.

.. warning:: Set a rate the vehicle can actually achieve. Requesting a rate the vehicle cannot deliver both shortens the timeout and leaves the rotation slower than requested, so the flip is likely to be abandoned part way around, returning to the previous flight mode at whatever attitude it had reached.
