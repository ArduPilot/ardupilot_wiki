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

