.. _flight-modes:

============
Flight Modes
============

This article provides an overview and links to the available flight modes
for Blimp.

Overview
========

Blimp has 5 built-in flight modes.

Flight modes are controlled through the radio (via a :ref:`transmitter switch <common-rc-transmitter-flight-mode-configuration>`), or using commands from a ground station (GCS) or
companion computer.

The table below shows for each flight mode whether it provides altitude or position control, and whether it requires valid position information from a sensor (typically a GPS) in order to arm or switch into this mode.

.. raw:: html
 
    <table border="1" class="docutils">
    <tr><th>Mode</th><th>Alt Ctrl</th><th>Pos Ctrl</th><th>Pos Sensor Required</th><th>Summary</th></tr>
    <tr><td>Land</td><td>-</td><td>-</td><td>No</td><td>Stops all actuators.</td></tr>
    <tr><td>Manual</td><td>m</td><td>m</td><td>No</td><td>Manual output via motor/servo mixer.</td></tr>
    <tr><td>Velocity</td><td>s</td><td>s</td><td>Yes</td><td>Stick positions set a desired velocity. Mostly intended for tuning.</td></tr>
    <tr><td>Loiter</td><td>s</td><td>s</td><td>Yes</td><td>Holds altitude and position.</td></tr>
    <tr><td>RTL</td><td>s</td><td>s</td><td>Yes</td><td>Returns to position/altitude set when first armed (Home) and sets 0deg YAW</td></tr>
    </table>

.. raw:: html
 
    <table border="1" class="docutils">
    <tr><th>Symbol</th><th>Definition</th></tr>
    <tr><td>m</td><td>Manual control</td><tr>
    <tr><td>s</td><td>Pilot controls desired position/velocity to controller</td></tr>
    </table>

Recommended Flight Modes
========================

It is best to start with manual mode to ensure the the actuators and rc controller have been set up correctly (i.e. pushing the pitch stick forward does make the blimp move forward).

Once this is confirmed, you can switch into VELOCITY mode. This uses only the velocity PID controller, thus allowing tuning this controller using the VEL* parameters. 
Test whether centering the sticks results in a standstill and no oscillation. Also test that the blimp reaches a set velocity reasonably quickly. 

After this stage, you can switch into LOITER mode and check its performance. Generally the position controller should need less tuning (using the POS* parameters), but some tuning may still be needed.

LAND mode stops all servo outputs so that the Blimp will stop moving. It can also be used as a pilot selected mode to save battery if the blimp needs to wait. Note that since the blimp is still floating, it is likely to drift with the wind, though it is recommended to have the blimp slightly negatively buoyant so that the blimp will also go down and "land" when in this mode.

Most transmitters can be setup to provide a 3 position switch that can be set up to quickly switch between the most-used flight modes but you can find instructions :ref:`here for setting up a 6-position flight mode switch <common-rc-transmitter-flight-mode-configuration>`.

GNSS Receiver ("GPS") Dependency
================================

Flight modes that use positioning data require valid position prior to takeoff. When using GPS, to verify if your autopilot has acquired GPS lock,
connect to a ground station or consult your autopilot's hardware
overview page to see the LED indication for GPS lock.

Below is a summary of position identification dependency for Copter flight modes. Most often this position information is obtained via a GPS, but other
position sensors, such as 3D cameras or beacons, may be used and would need to provide a valid location, for those modes requiring it, prior to arming.

Requires valid position prior to takeoff:

-  VELOCITY
-  LOITER

Do not require position information:

-  MANUAL
-  LAND

Pilot Control
=============

Pilot control in each mode (Except LAND in which there is no pilot control and all servo outputs are stopped) , if the default ``RC_MAP_xxxx`` parameters are used, is as follows:

==================    =================
TRANSMITTER STICK     CONTROL EFFECT
==================    =================
ROLL                  MANUAL: Lateral movement,
                      VELOCITY: Full throw attempts to obtain :ref:`MAX_VEL_XY<MAX_VEL_XY>` m/s laterally,
                      LOITER: Full throw attempts to obtain :ref:`MAX_POS_XY<MAX_POS_XY>` m/s laterally
PITCH                 MANUAL: Fore/Aft movement,
                      VELOCITY: Full throw attempts to obtain :ref:`MAX_VEL_XY<MAX_VEL_XY>` m/s  fore/aft,
                      LOITER: Full throw attempts to obtain :ref:`MAX_POS_XY<MAX_POS_XY>` m/s  fore/aft
YAW                   MANUAL: Yaw,
                      VELOCITY: Full throw attempts to increase/decrease heading :ref:`MAX_VEL_YAW<MAX_VEL_YAW>` radians/s  fore/aft,
                      LOITER: Full throw attempts to increase/decrease heading :ref:`MAX_POS_YAW<MAX_POS_XY>` radians/s  fore/aft
THROTTLE              MANUAL: Ascend/Descend,
                      VELOCITY: Full throw attempts to increase/decrease altitude :ref:`MAX_VEL_Z<MAX_VEL_Z>` m/s,
                      LOITER: Full throw attempts to increase/decrease heading :ref:`MAX_POS_Z<MAX_POS_Z>` m/s 
==================    =================

