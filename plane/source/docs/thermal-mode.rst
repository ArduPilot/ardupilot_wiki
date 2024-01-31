.. _thermal-mode:

============
THERMAL Mode
============

If SOARING is enabled (:ref:`SOAR_ENABLE<SOAR_ENABLE>` = 1 and it has not been disabled by an RC switch (``RCx_OPTION =88``), this mode will idle the throttle, and begin circling searching for vertical lift.

This is the mode that is used for autonomous soaring (See :ref:`soaring-4_1`) and is automatically entered if a thermal is detected during the gliding portion of SOARING flight, and will begin circling searching for the lift center.

THERMAL mode is exited under the following conditions:

   - :ref:`SOAR_ALT_MAX<SOAR_ALT_MAX>` is reached.
   - :ref:`SOAR_ALT_MIN<SOAR_ALT_MIN>` is reached.
   - Flight mode is changed by the pilot.
   - The estimate of achievable climb rate falls below :ref:`SOAR_VSPEED<SOAR_VSPEED>`, and 
     thermalling has lasted at least :ref:`SOAR_MIN_THML_S<SOAR_MIN_THML_S>` seconds.
   - The aircraft drifts more than :ref:`SOAR_MAX_DRIFT<SOAR_MAX_DRIFT>` - see :ref:`Limit maximum distance from home<soaring_maximum-distance-from-home>`

The flight mode will be returned to whatever it was before THERMAL was triggered.

This mode can be triggered by the pilot from any mode (like FBWA) using the GCS or RC transmitter, in which case THERMAL mode will begin searching for lift until THERMAL mode exit conditions are met or the pilot commands a mode change

While in THERMAL mode, the airspeed target is set to :ref:`SOAR_THML_ARSPD<SOAR_THML_ARSPD>` (meters/sec) if its non-zero, otherwise, if "0" (default) it will be set to :ref:`AIRSPEED_CRUISE<AIRSPEED_CRUISE>`.