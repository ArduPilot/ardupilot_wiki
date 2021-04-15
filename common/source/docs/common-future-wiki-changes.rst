.. _common-future-wiki-changes:

==========================================
Future Wiki Changes (in Master branch now)
==========================================



Serial Port Protocol Options
----------------------------

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Value</th>
   <th>Protocol</th>
   </tr>
   
   <tr>
   <td>26</td>
   <td>

Runcam see :ref:`common-camera-runcam` 

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>28</td>
   <td>

Scripting see :ref:`common-lua-scripts` 

.. raw:: html

   </td>
   </tr>
   
   </tbody>
   </table>

Output Mapping Page under QuadPlane Functions
---------------------------------------------

+--------------------------------+----+-----------------------------------------------------------------+
|      Tilt Motor Rear           | 45 |    QuadPlane                                                    |
+--------------------------------+----+-----------------------------------------------------------------+
|      Tilt Motor Rear Left      | 46 |    QuadPlane                                                    |
+--------------------------------+----+-----------------------------------------------------------------+
|      Tilt Motor Rear Right     | 47 |    QuadPlane                                                    |
+--------------------------------+----+-----------------------------------------------------------------+

- change title of explanations from "Motor Tilt/ Tilt Motor Left/ Tilt Motor Right" to "Tilt Motor/ Tilt Motor Left/ Tilt Motor Right/ Tilt Motor Rear/ Tilt Motor Rear Left/ Tilt Motor Rear Right"


RC Options Page
---------------

Add to table:

=================================       =========
:ref:`RC_OPTIONS<RC_OPTIONS>` bit       Function
=================================       =========
4                                       Log RC raw RC input bytes for serial protocols
5                                       Require Throttle input at idle position in order to arm
6                                       Allows arming if the rudder,elevator, or aileron
                                        stick is not neutral
7                                       Allow Aux Switches to honor the ``RCx_REVERSED`` parameter
=================================       =========


Add section at end:


There is also an :ref:`RC_PROTOCOLS<RC_PROTOCOLS>` bitmask that can be used to restrict which RC protocols are detected and used. This is useful in cases where the RC protocol autodetection fails and an incorrect RC protocol handler is chosen. This is rare, but if you do find it happens then you can lock in a single RC protocol that can be detected and used with this parameter.

Autopilot Output Mapping
------------------------

add to MISC Functions table:

+--------------------------------+----+---------------------------------------+
|       Function                 | ID |        Available in:                  |
+--------------------------------+----+---------------------------------------+
| Output SERVOn_MIN PWM value    |134 |    Plane, Copter, Rover               |
+--------------------------------+----+---------------------------------------+
| Output SERVOn_TRIM PWM value   |135 |    Plane, Copter, Rover               |
+--------------------------------+----+---------------------------------------+
| Output SERVOn_MAX PWM value    |136 |    Plane, Copter, Rover               |
+--------------------------------+----+---------------------------------------+

LUA Scripting (common-lua-scripts)
----------------------------------

see `Wiki PR #2839 <https://github.com/ArduPilot/ardupilot_wiki/pull/2839>`__  many new bindings

add to "Getting Started": 

- Scripts which require no user editing before use (Applets) can be found `here <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Scripting/applets>`_ . Each of these have an .md file of the same name detailing its capabilities, use and setup. For example, there is a script to allow a user to change a SmartAudio capable video transmitter's output power level from a transmitter channel and set its power-up value via parameter.
- Up to 8 RC channels can be assigned as scripting inputs/controls using the``RCX_OPTION`` = "300-307" options. In addition, four dedicated script parameters are avaliable: :ref:`SCR_USER1<SCR_USER1>` thru :ref:`SCR_USER4<SCR_USER4>` and are accessed with the same method as any other parameter, but these are reserved for script use.


Logging (common-downloading-and-analyzing-data-logs-in-mission-planner)
-----------------------------------------------------------------------

:ref:`LOG_FILE_MB_FREE<LOG_FILE_MB_FREE>` : This parameter sets the minimum free space on the logging media before logging begins. If this is not available, then older logs will be deleted to provide it during initialization. Default is 500MB.

Managing Gyro Noise with the Static Notch and Dynamic Harmonic Notch Filters(common-imu-notch-filtering)
--------------------------------------------------------------------------------------------------------

see `Wiki PR #2901 < <https://github.com/ArduPilot/ardupilot_wiki/pull/2901>`__ improved notch filtering operation

SRXL2 and CRSF RX Protocol Additions
------------------------------------

see `Wiki PR #2905 <https://github.com/ArduPilot/ardupilot_wiki/pull/2905>`__

update Common-RC-Systems and Common-Autopilot-Wiring pages with links to new pages


Advanced Setup Page
-------------------
Add link to :ref:`EKF3 Affinity and Lane Switching <common-ek3-affinity-lane-switching>`.

Firmware Limitations Section (common-autopilots.rst)
----------------------------------------------------

include in the note that OSD-base parameter editing not available on KakuteF7 and OmnibusF7V2 boards.
include in the note that ADSB is not included in 1MB boards.

MSP Protocol and OSD
--------------------

Add link to MSP (MultiWii Serial Protocol) <common-msp-overview> on Telemetry Landing Page, and add to OSD page:

"MSP protocol allows MSP compatible goggles (like the DJI Air system) to overlay telemetry data directly in the goggle display."
and link in TOC to : MSP OSD <common-msp-osd-overview>

Integrated OSD page
-------------------

OSD enhancements : `OSD Call Sign and Fonts <https://github.com/ArduPilot/ardupilot_wiki/pull/3173>`_

BiDirectional DSHOT and BLHeli_S Passthrough
--------------------------------------------

See `Bi-Directional DShot <https://github.com/ArduPilot/ardupilot_wiki/pull/3329/files>`__

PLANE
=====

On Quadplane Flying Page:
-------------------------

Change to WARNING box:

This can be managed somewhat with manual throttle control when manually transitioning, but in AUTO mode, a VTOL to fixed wing transition is currently done with :ref:`TKOFF_THR_MAX<TKOFF_THR_MAX>` p on the forward motor until transition is complete, so very high currents can be experienced..

Changes to Assisted Fixed-Wing Flight:

- change From: "To enable quad assistance you should set Q_ASSIST_SPEED parameter to the airspeed below which you want assistance." To : VTOL motor assistance is enabled if :ref:`Q_ASSIST_SPEED<Q_ASSIST_SPEED>`, :ref:`Q_ASSIST_ALT<Q_ASSIST_ALT>` , or :ref:`Q_ASSIST_ANGLE<Q_ASSIST_ANGLE>` are non-zero.
- remove: "The attitude assistance will only be used if Q_ASSIST_SPEED greater than zero."
- add after Q_ASSIST_ALT paragraph: Assistance can also be enabled, disabled, or forced by setting an RC switch to ``RCx_OPTION`` = 82. If that channel is below  1200us (LOW), then assistance is unconditionally disabled, if above 1800us, (HIGH) then assistance is always enabled. Fot other RC values, assistance will be enabled as explained above.

Assistance can also be forced active all the time by setting :ref:`Q_OPTIONS<Q_OPTIONS>` bit 7 to "1". For Tailsitters, assistance for tailsitters can be limited only to VTOL motors by by setting :ref:`Q_OPTIONS<Q_OPTIONS>` bit 8 to "1". This can increase stabiity during assistance by not using the copter style pid gains on the flying surfaces as well as the VTOL motors, or for use with copter tailsitters without servo controlled flying surfaces.

- add .. note:: Assistance is available for all QuadPlane frame types except the single motor and non-tilt dual motor tailsitter frames.

Assistance will be activated :ref:`Q_ASSIST_DELAY<Q_ASSIST_DELAY>` after any of the above enabling thresholds are reached.

Under the Hybrid RTL section:

change this sentence: "To enable this type of hybrid RTL mode you need to set the :ref:`Q_RTL_MODE<Q_RTL_MODE>` parameter to 1." to "To enable this type of hybrid RTL mode you need to set the :ref:`Q_RTL_MODE<Q_RTL_MODE>` parameter to 1 or 2." and add a trailing subheader: Q_RTL_MODE=1. Then at the end of the section, add a subhead: Q_RTL_MODE=2 and this - "Setting :ref:`Q_RTL_MODE<Q_RTL_MODE>` to 2 resuls in behaviour similar to above, but with the vehicle returning like normal fixed wing RTL until it reaches :ref:`Q_FW_LND_APR_RAD<Q_FW_LND_APR_RAD>`, then loitering in fixed wing mode to :ref:`Q_RTL_ALT<Q_RTL_ALT>` altitude, and then exiting facing the wind and executing a QRTL to the home position. Be sure the loiter portion is set up to clear any obstacles."

Add section:

Manual Forward Throttle in VTOL Modes
=====================================

By setting an RC channel option (``RCx_OPTION``) to "209", that channel can provide a separate throttle input to the forward motor(s) in QSTABILIZE, QACRO, and QHOVER VTOL modes. This allows forward movement without having to tilt the QuadPlane forward requiring throttle stick repositioning in QSTABILIZE and QACRO to maintain altitude, and present more forward flat plate resistance to forward movement in all modes.

On QuadPlane Parameters page:
-----------------------------

-add to Q_OPTIONS description:

-  bit 6, if set, will enforce the ICE idle governor even in MANUAL mode.
-  bit 7, if set, will force QASSIST to be active at all times in VTOL modes. See :ref:`Assisted Fixed-Wing Flight<assisted_fixed_wing_flight>`.
-  bit 8, if set, QASSIST will only affect VTOL motors. If not set, QAssist will also use flying surfaces to stabilize(:ref:`Assisted Fixed-Wing Flight<assisted_fixed_wing_flight>` ).
-  bit 9, if set, will enable AirMode (:ref:`airmode`) if armed via an RC switch. See :ref:`Auxiliary Functions<common-auxiliary-functions>` option value 41.
-  bit 10, if set, will allow the tilt servos to move with rudder input in vectored tilt setups while disarmed to determine range of motion.
-  bit 11, if set, will delay VTOL motor spin up until 2 seconds after arming.
-  bit 12, if set, disable speed based Qassist when using synthetic airspeed
-  bit 13, if set, will disable Ground Effect Compensation

On QHOVER mode page:
--------------------

under Controls section: change - "When the stick is completely down the QuadPlane will descend at :ref:`Q_VELZ_MAX<Q_VELZ_MAX>` and if at the very top it will climb by :ref:`Q_VELZ_MAX<Q_VELZ_MAX>`." to "When the stick is completely down the QuadPlane will descend at :ref:`Q_VELZ_MAX_DN<Q_VELZ_MAX_DN>` and if at the very top it will climb by :ref:`Q_VELZ_MAX<Q_VELZ_MAX>`."

On QuadPlane Auto Mode page:
----------------------------

Until Return to Launch section, add: Setting :ref:`Q_RTL_MODE<Q_RTL_MODE>` to 2 resuls in behaviour similar to the option for MAV_NAV_VTOL_LAND above, with the vehicle returning like normal fixed wing RTL until it reaches :ref:`Q_FW_LND_APR_RAD<Q_FW_LND_APR_RAD>`, then loitering to :ref:`Q_RTL_ALT<Q_RTL_ALT>`  altitude, and then exiting facing the wind and executing a QRTL to the home position. Be sure the loiter portion is set up to clear any obstacles.

On Terrain Following Page:
--------------------------

In Flight Modes Section change:

"Set :ref:`TERRAIN_FOLLOW<TERRAIN_FOLLOW>` to 1 to enable terrain following in those modes." to read "Setting the bitmask in :ref:`TERRAIN_FOLLOW<TERRAIN_FOLLOW>` determines which altitude controlled modes terrain following is active. For example, setting it to "10" enables following in FBWB and AUTO."


At the end of Flight Modes section:

Terrain Following in CRUISE and FBWB modes can be disabled with an RC switch assigned ``RCx_OPTION`` = 86. When enabling (<1200us) or disabling (>1800us) terrain following with the switch, the present altitude will be the target set point either above terrain, or home, respectively. The target altitude can be changed as normal with elevator whether the altitude reference being used is above home or above terrain.

On Dspoiler Page:
-----------------

Add at bottom of page:

Crow Mode Switch
================

If Differential Spoilers are used, setting an ``RCx_OPTIONS`` channel to "87" will allow the control of CROW aileron (outer spoilers) operation. 

- HIGH position: No change to CROW deflection amount or use of progressive crow.
- MIDDLE position: force progressive crow, assuming :ref:`DSPOILER_CROW_W1<DSPOILER_CROW_W1>` is non-zero, even if :ref:`DSPOILER_OPTS <DSPOILER_OPTS>` bit 2 is zero.
- LOW position: effectively sets :ref:`DSPOILER_CROW_W1<DSPOILER_CROW_W1>` to zero. Only inner spoilers move with FLAP channel, ie normal flaps.

This allows live changes to CROW operation on approaches so that speed braking and descent rates can be changed during the approach.

On Automatic Landing page:
--------------------------

- add note in Reverse-Thrust Landing section: 

.. note:: Airbrakes can also be automatically deployed during reverse thrust operation. See :ref:`Airbrakes<airbrakes-on-plane>`.

On Tilt Rotors and Tailsitters pages:
-------------------------------------

add note:

For landing in fixed wing, manual throttle controlled modes, there is an ``RCx_OPTION`` (89) that will force the tilt servos upright, force idle throttle, and optionally force the pitch to target :ref:`LAND_PITCH_CD<LAND_PITCH_CD>` for flaring to the normal fixed wing landing. This allows intentional or emergency fixed wing landings in MANUAL, ACRO, STABILIZE, and FBWA modes without the risk of a prop strike in configurations where this could occur otherwise.

On Tilt Rotors page:
--------------------

- add note and change table to:

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Tilt Control</th><th>SERVOn_FUNCTION</th></tr>
   <tr><td>Tilt Motors Rear</td><td>45</td></tr>
   <tr><td>Tilt Motor Rear Left</td><td>46</td></tr>
   <tr><td>Tilt Motor Rear Right</td><td>47</td></tr>
   <tr><td>Tilt Motor</td><td>41</td></tr>
   <tr><td>Tilt Motor Left</td><td>75</td></tr>
   <tr><td>Tilt Motor Right</td><td>76</td></tr>
   </table>

.. note:: For vectored yaw applications, the right and left tilt servos would be used for front and/or back.

On Tailsitters page, under Vectored Thrust:
-------------------------------------------

add note:

To allow vectored thrust QuadPlanes to land in the Fixed Wing (FW) stance in non-throttled controlled modes (in case of low battery level or emergency), an RC channel option, ``RCx_OPTION = 89``. This forces up the motor's tilts while still in FW non-throttled control modes (FBWA, MANUAL, STABILIZE, and ACRO) to avoid prop strikes upon ground contact when landing.

change note about Copter Tailsitters to read:

.. note:: in firmware versions previous to 4.1, CopterMotor Tailsitters did not use any yaw torque control. Roll (with respect to plane body) is only controlled by the flying surface (ailerons or elevons). Now QUAD PLUS and X frames have yaw control via motors, and frame types 16 and 17 are added that have no torque yaw control, as previous versions of PLUS and X did.

On Tailsitters page, under Tailsitter Configuration:
----------------------------------------------------

add frame types 16 and 17 to table of supported frame types for CopterMotor No Yaw Torque Tailsitters and add new diagrams with motor rotation matching Copter for yaw torque controlled plus (0) and X (1) frames.

.. image:: ../../../plane/source/images/x-copter-yawtorque-quadplane.jpg

.. image:: ../../../plane/source/images/plus-copter-yawtorque-quadplane.jpg

add note:

.. note:: it is possible to have a CopterMotor Tailsitter using no fixed wing control surfaces, ie basically a quadcopter with a wing. For that configuration, all Copter motors would be set to be active in fixed wing modes via :ref:`Q_TAILSIT_MOTMX<Q_TAILSIT_MOTMX>` and :ref:`Q_OPTIONS<Q_OPTIONS>` bitmask would have bit 7 (Force QASSIST) set to have QASSIST active in all modes.


On Airspeed Calibration Page
----------------------------

Add section at end:

Miss-calibration Safeguards
===========================

In order to help prevent Airspeed sensor use when its been miss-calibrated either during ground static calibration during the power up sequence, or by accidental parameter changes to offset or ratio, three parameters are available. If the ground speed is consistently lower than the reported airspeed for a few seconds by :ref:`ARSPD_WIND_MAX<ARSPD_WIND_MAX>`, i.e. the apparent wind speed is greater than that amount, the sensor can be disabled to avoid erroneous reporting. It can be allowed to re-enable if the apparent wind falls back below that value. These actions are controlled by :ref:`ARSPD_OPTIONs<ARSPD_OPTIONs>`.

You can also send a warning to the Ground Control Station if the apparent wind exceeds :ref:`ARSPD_WIND_WARN<ARSPD_WIND_WARN>`. This can be used instead of, or together with the above.

On Quadplane Tips Page:
-----------------------

Under Tilt Rotor Servo Setup, add:

Note that setting :ref:`Q_OPTIONS<Q_OPTIONS>` bit 10 (Disarmed Yaw Tilt) allows the motors to tilt in response to rudder input while disarmed to facilitate adjustment of parameters.

On Flight Options Page:
 add to table

=====================================   ======================
:ref:`FLIGHT_OPTIONS<FLIGHT_OPTIONS>`   Function
=====================================   ======================
4                                       Climb to :ref:`ALT_HOLD_RTL<ALT_HOLD_RTL>` altitude before turning toward home in RTL
=====================================   ======================

-----------------------------------------------------

Copter
======

On BendyRuler page:
-------------------

- replace OA_LOOKAHEAD label with :ref:`OA_BR_LOOKAHEAD<OA_BR_LOOKAHEAD>`
- add these params under configuration:
    - :ref:`OA_BR_CONT_RATIO<OA_BR_CONT_RATIO>` : BendyRuler will avoid changing bearing unless ratio of previous margin from obstacle (or fence) to present calculated margin is atleast this much.
    - :ref:`OA_BR_CONT_ANGLE<OA_BR_CONT_ANGLE>` : BendyRuler will resist changing current bearing if the change in bearing is over this angle

On AIRMODE  page:
-----------------

AIRMODE can also be set to be active without using the ``RCx_OPTION`` ARM/DISARM switch to arm. Setting an RC channel to ``RCx_OPTION`` = 84, allows enabling or disabling AIRMODE in ACRO and STABILIZE modes directly. In addition, setting bit 0 of :ref:`ACRO_OPTIONS<ACRO_OPTIONS>` will activate AIRMODE in those modes all the time.

On AUTO mode page:
------------------

Add note in Control section:

The :ref:`AUTO_OPTIONS<AUTO_OPTIONS>` parameter can be used to alter this behaviour, allowing arming while in AUTO mode, and/or, allowing a mission takeoff command to start upon AUTO mode entry, even if the thorttle has not been raised.

On ACRO mode page:
------------------

in the first paragraph add:
Pure Rate mode stabilization, utilizing only the gyros with no attitude feedback from the accelerometers, can be forced by setting bit 1 of :ref:`ACRO_OPTIONS<ACRO_OPTIONS>` to one. Attitude will still have an open loop correction applied in this mode, similar to "heading hold" tail gyros, but attitude can drift over time.


On Circle mode page:
--------------------

Add info on CIRCLE_OPTIONS parameter which replaces the CIRCLE_CONTROL enable parameter:

When bit 0 is set of the :ref:`CIRCLE_OPTIONS<CIRCLE_OPTIONS>` parameter the pilot can adjust circle's radius and angular velocity with the control sticks:
When bit 1 is set of the :ref:`CIRCLE_OPTIONS<CIRCLE_OPTIONS>` parameter the Copter will face the direction of travel as it circles, otherwise, the Copter will point its nose at the center of the circle as it orbits.
When bit 2 is set of the :ref:`CIRCLE_OPTIONS<CIRCLE_OPTIONS>` parameter the circle's center position will set upon mode entry at the current location, rather than on the perimeter with the center in front of the Copter at the start.

Rover
=====

add Fence section to Sailboat configuration page:

Fences
------

Sailboats behave in the same manner as other Rovers regarding fence operation and breach failsafe actions. However, unlike other Rovers, which slow as they approach a fence boundary, Sailboats will just attempt to tack away from the boundary since they have no speed controller, as such.


