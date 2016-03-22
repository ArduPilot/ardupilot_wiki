.. _channel-7-and-8-options:

===========================
Auxiliary Function Switches
===========================

This article shows how to set up which features are invoked from the
transmitter's auxiliary function switches (Up to/including AC3.2.1 you
could only set the Channel 7 and 8 options, From AC3.3 you can set
options for channels 7 - 12).

.. note::

   AC3.3 introduces support for auxiliary functions on channels 9 ~
   12.

Configuration
=============

By default the auxiliary switches are disabled but you can assign
features using the *Mission Planner*'s **Config/Tuning \| Extended
Tuning** screen or by setting the ``CH7_OPT`` (or ``CH8_OPT``) parameter
directly.

.. image:: ../../../images/MPchan7options.jpg
    :target: ../_images/MPchan7options.jpg

Here are the options:

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Option</th>
   <th>Description</th>
   </tr>
   <tr>
   <td><strong>Flip</strong></td>
   <td>Vehicle will flip is on roll axis. Vehicle will rise for 1 second and
   then rapidly roll. The vehicle will not flip again until the switch is
   brought low and back to high. Give yourself at least 10m of altitude
   before trying flip for the first time!
   </td>
   </tr>
   <tr>
   <td><strong>Simple Mode</strong></td>
   <td>
   
The copter moves left, right, forward and back as if it were facing in
the same direction as when it took off regardless of the copter's
heading. :ref:`See here for more details <simpleandsuper-simple-modes>`.

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>RTL</strong></td>
   <td>

   
Return-to-Launch. The vehicle will first climb to a safe altitude
(defined by ``RTL_ALT`` parameter), then return home, pause over home
for 5 seconds (configurable by ``RTL_LOIT_TIME`` parameter) then descend
to 2m or land (configurable by ``RTL_ALT_FINAL`` parameter).

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>Save Trim</strong></td>
   <td>
   

Adjust the vehicle level position using the current roll and pitch stick
inputs. See details `here <autotrim>`__.</td>

.. raw:: html

   </tr>
   <tr>
   <td><strong>Save WP</strong></td>
   <td>Save the current location (including altitude) as a waypoint in the
   mission. If in AUTO mode no waypoint will be saved, instead the vehicle will RTL</td>
   </tr>
   <tr>
   <td><strong>Multi Mode</strong></td>
   <td>
   

.. note::

   This option has been removed from AC3.3 (and higher).

Ch6 tuning knob position controls which of the following 3 functions is
invoked when ch7 or ch8 switch put in the on position.

-  Ch6 <1200 : Flip
-  Ch6 1201 ~ 1799 : RTL
-  Ch6 > 1800 : Save Waypoint

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>Camera Trigger</strong></td>
   <td>

Camera shutter will be activated. See more details
:ref:`here <common-camera-shutter-with-servo>`.

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>Sonar</strong></td>
   <td>Sonar is disabled when switch is in low position, enabled when it high position.</td>
   </tr>
   <tr>
   <td><strong>Fence</strong></td>
   <td>Fence is disabled when switch is in low position, enabled when it high position.</td>
   </tr>
   <tr>
   <td><strong>ResetToArmedYaw</strong></td>
   <td>Vehicle will turn to face the same direction that it was facing when it
   was first armed (e.g. at take-off). An alternative to simple mode when
   you've lost orientation and need to bring the craft home.</td>
   </tr>
   <tr>
   <td><strong>Super Simple Mode</strong></td>
   <td>

Similar to simple mode except controls are relative to the home
position. Move the pitch stick forward and the copter will fly away from
home, pull pitch back and it will return. Move roll stick left or right
and the copter to orbit home regardless of the copter's actual
heading. :ref:`See here for more details <simpleandsuper-simple-modes>`.

If mapped to a three-position switch then the function will allow
Super-Simple **and Simple** modes to be enabled using the high and
middle switch positions, respectively (a two-position switch will
enable/disable Super Simple mode only).

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>

:ref:`Landing Gear <landing-gear>`

.. raw:: html

   </td>
   <td>Retracts/Deploys landing gear. (AC3.3 only)</td>
   </tr>
   <tr>
   <td><strong>Lost Copter Alarm</strong></td>
   <td>

Plays the `lost copter alarm <http://firmware.ardupilot.org/downloads/wiki/pixhawk_sound_files/LostCopter.wav>`__
though the buzzer (AC3.3 only)

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>Emergency Stop Motors</strong></td>
   <td>

Stops motors immediately
(`video <https://www.youtube.com/watch?v=-Db4u8LJE5w>`__). (AC3.3 only)

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>Motor Interlock</strong></td>
   <td>

Opposite of Emergency stop (above) in that switch must be ON for motors
to spin (`video <https://youtu.be/-Db4u8LJE5w?t=51>`__). (AC3.3 only)

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>Brake</strong></td>
   <td>

Invokes the :ref:`Brake flight mode <brake-mode>` when switch goes high.
Bringing switch back to low will return the vehicle to the mode
indicated by the ch5 flight mode switch. (AC3.3 only)

.. raw:: html

   </td>
   </tr>
   </tbody>
   </table>

Check Ch7 & Ch8 PWM values reach 1800
=====================================

Your channel 7 and/or channel 8 pwm value must reach at least 1800 PWM
or the auxiliary function will not be activated. You can check that it
surpasses this threshold using the Mission Planner's radio config page.

.. image:: ../../../images/MPCH7checkPWMrange.jpg
    :target: ../_images/MPCH7checkPWMrange.jpg
