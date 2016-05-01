.. _hold-mode:

=========
HOLD Mode
=========

All Stop.

-  When you switch to Hold mode, the vehicle should stop with its wheels
   pointed straight ahead.
-  It is necessary to set your throttle (``RC3_Trim``) parameter to the
   correct position for your vehicles throttle off PWM.
-  If you have a reversing vehicle with a center neutral point set
   ``RC3_TRIM`` parameter to 1500: (May need adjustment for your
   vehicle).
-  If you have a non reversing vehicle RC3_TRIM can be left to it's
   default low (~1100 PWM) setting or adjusted to it if necessary.
-  In Auto mode if RC reception is lost, the controller will
   automatically switch to Hold mode.
-  If you enable Throttle Failsafe and set the Failsafe action to HOLD
   it will go to HOLD if radio reception is lost in all modes.
-  Normally you do not have to install this as a switch option.
-  If it failsafes to HOLD mode, switching to another mode and back will
   clear it.
