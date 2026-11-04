.. _flight-modes:

============
Flight Modes
============

This article provides an overview and links to the available flight modes
for Blimp.

Overview
========

Blimp has 7 built-in flight modes.

Flight modes are controlled through the radio (via a :ref:`transmitter switch <common-rc-transmitter-flight-mode-configuration>`), or using commands from a ground station (GCS) or
companion computer. The number in the table is the value to use in the ``FLTMODEx`` parameters (e.g. :ref:`FLTMODE1<FLTMODE1>`).

The table below shows for each flight mode whether it provides altitude or position control, and whether it requires valid position information from a sensor (typically a GPS) in order to arm or switch into this mode.

.. raw:: html

    <table border="1" class="docutils">
    <tr><th>Mode</th><th>Number</th><th>Alt Ctrl</th><th>Pos Ctrl</th><th>Pos Sensor Required</th><th>Summary</th></tr>
    <tr><td>Land</td><td>0</td><td>A</td><td>A</td><td>No</td><td>Descends slowly while holding position.</td></tr>
    <tr><td>Manual</td><td>1</td><td>m</td><td>m</td><td>No</td><td>Manual output via motor/servo mixer.</td></tr>
    <tr><td>Velocity</td><td>2</td><td>s</td><td>s</td><td>Yes</td><td>Stick positions set a desired velocity. Mostly intended for tuning.</td></tr>
    <tr><td>Loiter</td><td>3</td><td>s</td><td>s</td><td>Yes</td><td>Holds altitude and position.</td></tr>
    <tr><td>RTL</td><td>4</td><td>A</td><td>A</td><td>Yes</td><td>Returns to position/altitude set when first armed (Home) and sets 0deg YAW</td></tr>
    <tr><td>Auto</td><td>5</td><td>A</td><td>A</td><td>Yes</td><td>Flies a mission of waypoints.</td></tr>
    <tr><td>Hold</td><td>6</td><td>-</td><td>-</td><td>No</td><td>Stops all actuators.</td></tr>
    </table>

.. raw:: html

    <table border="1" class="docutils">
    <tr><th>Symbol</th><th>Definition</th></tr>
    <tr><td>m</td><td>Manual control</td></tr>
    <tr><td>s</td><td>Pilot controls desired position/velocity to controller</td></tr>
    <tr><td>A</td><td>Automatic control</td></tr>
    </table>

Recommended Flight Modes
========================

It is best to start with manual mode to ensure the the actuators and rc controller have been set up correctly (i.e. pushing the pitch stick forward does make the blimp move forward).

Once this is confirmed, you can switch into VELOCITY mode. This uses only the velocity PID controllers, thus allowing tuning these controllers using the ``LOIT_VELX_``, ``LOIT_VELY_``, ``LOIT_VELZ_`` and ``LOIT_VELYAW_`` parameters (e.g. :ref:`LOIT_VELX_P<LOIT_VELX_P>`).
Test whether centering the sticks results in a standstill and no oscillation. Also test that the blimp reaches a set velocity reasonably quickly.

After this stage, you can switch into LOITER mode and check its performance. Generally the position controllers should need less tuning (using the ``LOIT_POSX_``, ``LOIT_POSY_``, ``LOIT_POSZ_`` and ``LOIT_POSYAW_`` parameters, e.g. :ref:`LOIT_POSX_P<LOIT_POSX_P>`), but some tuning may still be needed.

Other parameters that affect the VELOCITY and LOITER controllers are:

- :ref:`LOIT_DIS_MASK<LOIT_DIS_MASK>`: disables (sets to zero) one or more of the four output axes, which can be useful while tuning one axis at a time
- :ref:`LOIT_PID_DZ<LOIT_PID_DZ>`: outputs zero thrust when the blimp is within this distance of the target position
- :ref:`LOIT_POS_LAG<LOIT_POS_LAG>`: number of seconds' worth of travel that the actual position can be behind the target position

HOLD mode stops all actuator outputs so that the Blimp will stop moving. It can be used as a pilot selected mode to save battery if the blimp needs to wait. Note that since the blimp is still floating, it is likely to drift with the wind, though it is recommended to have the blimp slightly negatively buoyant so that the blimp will also go down and "land" when in this mode.

LAND mode descends at half of :ref:`LOIT_MAX_VELZ<LOIT_MAX_VELZ>` while holding horizontal position. If there is no position estimate, it instead outputs half downward thrust with no horizontal control.

Most transmitters can be setup to provide a 3 position switch that can be set up to quickly switch between the most-used flight modes but you can find instructions :ref:`here for setting up a 6-position flight mode switch <common-rc-transmitter-flight-mode-configuration>`.

AUTO Mode
=========

AUTO mode flies the mission loaded into the autopilot. Currently only waypoint (``NAV_WAYPOINT``) commands are supported. Any other command is skipped with a "Command not supported" message. The mission starts once the vehicle has a good position estimate. At the end of the mission, the blimp holds position at the last waypoint.

Travel between waypoints uses S-curve path planning, controlled by:

- :ref:`WP_VEL<WP_VEL>`: maximum horizontal speed (vertical speed is limited by :ref:`LOIT_MAX_VELZ<LOIT_MAX_VELZ>`)
- :ref:`WP_ACCEL<WP_ACCEL>`: maximum acceleration
- :ref:`WP_RADIUS<WP_RADIUS>`: waypoint acceptance radius
- :ref:`WP_YAW_SPD<WP_YAW_SPD>`: yaw rate used to turn the blimp to face its direction of travel
- :ref:`WP_YAW_MIN_VEL<WP_YAW_MIN_VEL>`: minimum horizontal speed below which the blimp does not yaw towards its direction of travel

GNSS Receiver ("GPS") Dependency
================================

Flight modes that use positioning data require valid position prior to takeoff. When using GPS, to verify if your autopilot has acquired GPS lock,
connect to a ground station or consult your autopilot's hardware
overview page to see the LED indication for GPS lock.

Below is a summary of position identification dependency for Blimp flight modes. Most often this position information is obtained via a GPS, but other
position sensors, such as 3D cameras or beacons, may be used and would need to provide a valid location, for those modes requiring it, prior to arming.

Requires valid position prior to takeoff:

-  VELOCITY
-  LOITER
-  RTL
-  AUTO

Do not require position information:

-  MANUAL
-  LAND
-  HOLD

Pilot Control
=============

Pilot control in MANUAL, VELOCITY and LOITER, if the default ``RCMAP_xxxx`` parameters are used, is as follows:

==================    =================
TRANSMITTER STICK     CONTROL EFFECT
==================    =================
ROLL                  MANUAL: Lateral movement,
                      VELOCITY: Full throw attempts to obtain :ref:`LOIT_MAX_VELY<LOIT_MAX_VELY>` m/s laterally,
                      LOITER: Full throw attempts to obtain :ref:`LOIT_MAX_POSY<LOIT_MAX_POSY>` m/s laterally
PITCH                 MANUAL: Fore/Aft movement,
                      VELOCITY: Full throw attempts to obtain :ref:`LOIT_MAX_VELX<LOIT_MAX_VELX>` m/s  fore/aft,
                      LOITER: Full throw attempts to obtain :ref:`LOIT_MAX_POSX<LOIT_MAX_POSX>` m/s  fore/aft
YAW                   MANUAL: Yaw,
                      VELOCITY: Full throw attempts to increase/decrease heading :ref:`LOIT_MAX_VELYAW<LOIT_MAX_VELYAW>` radians/s,
                      LOITER: Full throw attempts to increase/decrease heading :ref:`LOIT_MAX_POSYAW<LOIT_MAX_POSYAW>` radians/s
THROTTLE              MANUAL: Ascend/Descend,
                      VELOCITY: Full throw attempts to increase/decrease altitude :ref:`LOIT_MAX_VELZ<LOIT_MAX_VELZ>` m/s,
                      LOITER: Full throw attempts to increase/decrease altitude :ref:`LOIT_MAX_POSZ<LOIT_MAX_POSZ>` m/s
==================    =================

In MANUAL mode, :ref:`MAX_MAN_THR<MAX_MAN_THR>` limits the commanded output, in addition to the overall :ref:`FINS_THR_MAX<FINS_THR_MAX>` limit.
