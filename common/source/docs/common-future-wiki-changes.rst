.. _common-future-wiki-changes:

==========================================
Future Wiki Changes (in Master branch now)
==========================================

COMMON WIKI PAGES
=================

RCx_OPTIONs on Auxillary Functions Page:
----------------------------------------


+----------------------+----------------------------+----------+---------+---------+
| **RCx_OPTION value** | **Feature Description**    |**Copter**|**Plane**|**Rover**|
+----------------------+----------------------------+----------+---------+---------+
|        78            | RunCam Control             |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        79            | RunCam OSD Control         |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        80            | Viso Align                 |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        81            | Disarm                     |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        82            | Q_Assist 3Pos Sw           |          |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        83            | ZIGZAG Auto                |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        84            | AIRMODE(not a flight mode) |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        85            | Generator                  |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        86            | Non Auto Terrain Follow    |          |    x    |         |
|                      | Disable                    |          |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        87            | CROW Mode Switch           |          |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        100           | Kill IMU1 (testing only!)  |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        101           | Kill IMU2 (testing only!)  |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        105           | GPS Disable Yaw            |    X     |    X    |    X    |
|                      | (testing only!)            |          |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        209           | Forward Throttle           |          |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        300-307       | Scripting RC channels      |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Option</th>
   <th>Description</th>
   </tr>

.. raw:: html

   </td>
   <tr>
   <td><strong>ZigZag Mode Learn Waypoints</strong></td>
   <td>

Sets zigzag point A and B. See :ref:`zigzag-mode` .

.. raw:: html

   </td>
   </tr>
   <tr>

   <td><strong>RunCam Control</strong></td>
   <td>

Allows starting and stopping video recording of compatible RunCam cameras. See :ref:`common-camera-runcam`

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>RunCam OSD Control</strong></td>
   <td>

Enables control of RunCam cameras OSDs. See :ref:`common-camera-runcam`

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>Viso Align</strong></td>
   <td>

Align T265 Visual Odometry camera attitude to vehicle's.

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>Disarm</strong></td>
   <td>

Disarm vehicle unconditionally and immediately. Unlike Emergency Stop Motors, which waits for :ref:`DISARM_DELAY<DISARM_DELAY>` in Copter.

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>Q_Assist 3Pos SW</strong></td>
   <td>

Low: disable Q_Assist entirely, Middle: Normal Q_Assist operation, High: Q_Assist active at all times. See Assisted Fixed Wing Flight section of :ref:`quadplane-flying`



.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>ZigZag Mode Auto Enable</strong></td>
   <td>

Enable automatic zigzag and sprayer in ZIGZAG mode. See :ref:`zigzag-mode`

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>AIRMODE (not a regular flight mode)</strong></td>
   <td>

Enables and disables AIRMODE feature. See :ref:`airmode`

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>GPS Disable Yaw</strong></td>
   <td>

Disables yaw for testing (advanced users only!)

.. raw:: html

   </td>
   </tr>
      <tr>
   <td><Generator</strong></td>
   <td>

Mode control for Richenpower Hybrid Power Generator

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>Non Auto Terrain Follow Disable</strong></td>
   <td>

Disables Terrain Following in CRUISE and FBWB modes

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>CROW Mode Switch</strong></td>
   <td>

Selects between different CROW aileron operating modes

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>Kill IMU1 </strong></td>
   <td>

Disables IMU1 for testing (advanced users only!)

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>Kill IMU2 </strong></td>
   <td>

Disables IMU2 for testing (advanced users only!)

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>Forward Throttle </strong></td>
   <td>

Manual forward motor throttle in QSTABILIZE, QACRO, and QHOVER modes

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>Scripting RC channels </strong></td>
   <td>

Allows reading a dedicated RC channel for script inputs

.. raw:: html

   </td>
   </tr>
   </tbody>
   </table>

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

LUA Scripting (common-lua-scripts)
----------------------------------

see `Wiki PR #2839 <https://github.com/ArduPilot/ardupilot_wiki/pull/2839>`__  many new bindings

add to "Getting Started": Up to 8 RC channels can be assigned as scripting inputs/controls using the``RCX_OPTION`` = "300-307" options. In addition, four dedicated script parameters are avaliable: :ref:`SCR_USER1<SCR_USER1>` thru :ref:`SCR_USER4<SCR_USER4>` and are accessed with the same method as any other parameter, but these are reserved for script use.


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


------------------------------------------------------

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

Add section:

Manual Forward Throttle in VTOL Modes
=====================================

By setting an RC channel option (``RCx_OPTION``) to "209", that channel can provide a separate throttle input to the forward motor(s) in QSTABILIZE, QACRO, and QHOVER VTOL modes. This allows forward movement without having to tilt the QuadPlane forward requiring throttle stick repositioning in QSTABILIZE and QACRO to maintain altitude, and present more forward flat plate resistance to forward movement in all modes.

On Terrain Following Page:
--------------------------

At the end of Flight Modes section:

Terrain Following in CRUISE and FBWB modes can be disabled with an RC switch assigned ``RCx_OPTION`` = 86. When enabling (<1200us) or disabling (>1800us) terrain following with the switch, the present altitude will be the target set point either above terrain, or home, respectively. The target altitude can be changed as normal with elevator whether the altitude refernce being used is above home or above terrain.

On Dspoiler Page:

Add at bottom of page:

Crow Mode Switch
================

If Differential Spoilers are used, setting an ``RCx_OPTIONS`` channel to "87" will allow the control of CROW aileron (outer spoilers) operation. 

- HIGH position: No change to CROW deflection amount or use of progressive crow.
- MIDDLE position: force progressive crow, assuming :ref:`DSPOILER_CROW_W1<DSPOILER_CROW_W1>` is non-zero, even if :ref:`DSPOILER_OPTS <DSPOILER_OPTS>` bit 2 is zero.
- LOW position: effectively sets :ref:`DSPOILER_CROW_W1<DSPOILER_CROW_W1>` to zero. Only inner spoilers move with FLAP channel, ie normal flaps.

This allows live changes to CROW operation on approaches so that speed braking and descent rates can be changed during the approach.


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

On ACRO mode page:
------------------

in the first paragraph add:
Pure Rate mode stabilization, utilizing only the gyros with no attitude feedback from the accelerometers, can be forced by setting bit 1 of :ref:`ACRO_OPTIONS<ACRO_OPTIONS>` to one. Attitude will still have an open loop correction applied in this mode, similar to "heading hold" tail gyros, but attitude can drift over time.

------------------------------------------------------

Rover
=====

add Fence section to Sailboat configuration page:

Fences
------

Sailboats behave in the same manner as other Rovers regarding fence operation and breach failsafe actions. However, unlike other Rovers, which slow as they approach a fence boundary, Sailboats will just attempt to tack away from the boundary since they have no speed controller, as such.

[copywiki destination="plane,copter,rover,dev"]
