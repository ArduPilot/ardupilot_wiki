.. _common-compass-setup-advanced:

======================
Advanced Compass Setup
======================

This article provides advanced guidance for how to setup the system compass(es) and advanced compass related features.

.. tip::

   Users who have only internal compasses or an external compass using the :ref:`UBlox GPS + Compass Module <common-installing-3dr-ublox-gps-compass-module>` (recommended) and have mounted it in the default orientation can usually perform a simple "Onboard Calibration" as described in :ref:`Compass Calibration <common-compass-calibration-in-mission-planner>`).

Overview
========

Accurately setting up the compass is critical because it is the primary
source of heading information. Without an accurate heading the vehicle
will not move in the correct direction in autopilot modes (i.e. AUTO,
LOITER, PosHold, RTL, etc). This can lead to circling (aka
"toiletbowling") or fly-aways.

ArduPilot currently allows up to three compasses to be connected. Only
one compass (specified using the ``COMPASS_PRIMARY`` parameter) is used
for navigation. While many autopilots have an internal compass or compasses, many
will instead use an external compass. This provides more reliable data
than an internal compass because of the separation from other
electronics. See :ref:`common-autopilots` for details about the specific autopilot to determine how many built-in compasses the autopilot may have, if any.

Most users will only need to select compass configuration and perform the :ref:`Basic Compass Calibration<common-compass-calibration-in-mission-planner>` but details are also given on the less-used  :ref:`CompassMot <common-compass-setup-advanced_compassmot_compensation_for_interference_from_the_power_wires_escs_and_motors>` and Manual Declination. Most of this configuration can be performed from the *Mission Planner*'s **Initial Setup \| Mandatory Hardware \| Compass** screen.  
Other ground stations may have similar features.

.. tip::

   The article is Copter-focused but the instructions are equally
   applicable to Plane and Rover (except where marked).

Configuration settings
======================

The *Mission Planner Compass Setup screen* can be found in menu
**Initial Setup \| Mandatory Hardware \| Compass** in the sidebar. This
screen is used for setting almost all compass configuration and tuning
parameters.

.. figure:: ../../../images/MissionPlanner_CompassCalibration_MainScreen.png
   :target: ../_images/MissionPlanner_CompassCalibration_MainScreen.png

   Mission Planner: Compass Calibration

Quick configuration for Pixhawk
-------------------------------

Mission Planner supports automatic configuration of almost all
parameters for the Pixhawk autopilot boards. If it is a Pixhawk autopilot, select the button **Pixhawk/PX4**. You may be prompted for a specific ArduPilot version.

Configuration for non-Pixhawk Autopilots
----------------------------------------

- First determine how many on-board compasses the autopilot includes and enable those compasses. For example, if two compasses are integrated on-board, check the "Use this compass" box for Compass 1 and 2, and make sure that "External compass" boxes are unchecked. If you chose not use one of the internal compasses, then do not check its "Use this compass" box.
- If using an external compass, check the appropriate the appropriate "Use this compass" box. It should be the one following the internal compasses, whether being used, or not. If your external compass is in a non-standard orientation, you can manually select the orientation (aligned with IMU orientation)in the combo box (change from ``ROTATION_NONE``). See the Checking Compass Orientation Section below. However, ArduPilot 4.0 and later firmware versions will automatically determine orientation when the Onboard Mag Calibration routine is run. When externally connected, the :ref:`COMPASS_ORIENT<COMPASS_ORIENT>` parameter is independent of the :ref:`AHRS_ORIENTATION<AHRS_ORIENTATION>` board orientation option.
- The :ref:`AHRS_ORIENTATION<AHRS_ORIENTATION>` must be set correctly for the compass calibration to be successful. In addition,the Accelerometer Calibration should be completed before the Compass Calibration.

Most users will then only need to press the **OnBoard Mag Calibration** button
and perform a :ref:`Onboard Calibration <onboard_calibration>`.

Checking Compass Orientation
----------------------------
-  Ensure your :ref:`AHRS_ORIENTATION<AHRS_ORIENTATION>` parameter is correct.  This will ensure that your internal compass' orientation will be correct
-  When rotating your aircraft through all axes each of the compasses should move in the same direction, and should be of approximately the same values

- Northern Hemisphere:
  - Z-component should be *positive*
  - when pitching the vehicle down, the X component should *increase* in value
  - when rolling the vehicle right, the Y component should *increase* in value

- Southern Hemisphere:
  - Z-component should be *negative*
  - when pitching the vehicle down, the X component should *decrease* in value
  - when rolling the vehicle right, the Y component should *decrease* in value

These should be correct for any on-board compasses, since the orientation is defined in the autopilots definition file for the firmware.

General settings
----------------

The *general settings* apply to all compasses connected to the autopilot
controller:

-  **Enable compasses**: determines whether whether (any) compasses are
   enabled. If enabled the autopilot will use the primary
   compass for heading data, otherwise the heading will be estimated
   from GPS. Enabling this checkbox corresponds to setting parameter
   ``MAG_ENABLE=1``.

   .. note::

      Compasses should always be enabled for Copter/Rover, but may be
         disabled (not recommended) for Plane.

-  **Primary Compass**: specifies which compass ArduPilot will use for
   heading data (only one compass is used for navigation). Normally this
   will be set to the first compass ("Compass1"). This selection list
   corresponds to setting the parameter ``COMPASS_PRIMARY`` to a value
   from 0 to 2 (compasses are 0 indexed, even though labelled in the
   screen from 1 to 3).
-  **Obtain declination automatically**: sets the declination based on
   lookup tables following GPS lock. Users can override this default
   behaviour; after deselecting the checkbox (``COMPASS_AUTODEC=0``)
   they can manually enter declination in ``COMPASS_DEC``.
-  **Automatically learn offsets**: See Automatic Offset Learning section below.

Compass specific settings
-------------------------

The settings that are specific to each compass are grouped together.
Some settings are only visible when the compass is enabled.

-  **Use this compass**: This checkbox enables a particular compass for
   use by the autopilot. Each checkbox corresponds to a ``COMPASS_USEx``
   parameter (where *x* is 0 to 2, depending on the compass).

   .. note::

      Even if multiple ``COMPASS_USEx`` parameters are set to 1, the
         autopilot will still only uses the primary compass
         (``COMPASS_PRIMARY``).

-  **Externally mounted**: Set whether or not a particular compass is
   externally mounted (corresponds to ``COMPASS_EXTERNAL=1``). If the
   compass is internal it uses the autopilot’s orientation
   (``AHRS_ORIENTATION``). If the compass is external, the orientation
   may differ from the autopilot (set using the selection list
   discussed next)
-  **Compass orientation**: sets the compass orientation for externally
   mounted compasses. The value is saved as a ``COMPASS_ORIENTx``
   parameter.

The OFFSETS (``COMPASS_OFFSx``) and and MOT (``COMPASS_MOT``) parameters
are populated by the live calibration and CompasMot procedures (see the
calibration sections below).


.. _common-compass-setup-advanced_live_calibration_of_offsets:

Live calibration of offsets
===========================

.. note:: This method is no longer recommended for recent versions of ArduPilot that support Onboard Calibration. See the basic calibration page :ref:`here<common-compass-calibration-in-mission-planner>`.

Live calibration calculates offsets to compensate for “hard iron”
distortions.

#. Click the **Live Calibration** button.

   A window should pop-up showing you the state of the live calibration.
   This shows a sphere for each compass with a red dot showing where the
   compass is pointing and six "white dot" targets around the sphere.
   You rotate the vehicle so that the red dot reaches each white dot and
   causes it to disappear.

   .. figure:: ../../../images/MissionPlanner_CompassCalibration_LiveCalibrationScreen.png
      :target: ../_images/MissionPlanner_CompassCalibration_LiveCalibrationScreen.png

      Mission Planner: Live Compass Calibration

   As you rotate the vehicle you will notice the red dot moves and
   (perhaps confusingly) the sphere itself also rotates. A colored trail
   is left behind wherever the compass has already been: high values (>
   400) will turn yellow and may indicate magnetic interference. Offsets
   > 600 will turn red and generate a warning.

#. Hold the vehicle in the air and rotate it slowly so that each side
   (front, back, left, right, top and bottom) points down towards the
   earth for a few seconds in turn.

   .. figure:: ../../../images/accel-calib-positions-e1376083327116.jpg
      :target: ../_images/accel-calib-positions-e1376083327116.jpg

      Compass Calibration Positions (shown for Copter, but true for all vehicles)

#. The calibration will automatically complete when it has data for all
   the positions. At this point, another window will pop up telling you
   that it is saving the newly calculated offsets. These are displayed
   on the main screen below each associated compass.

   .. note::

      In Copter-3.2.1 and later offsets are considered acceptable
         provided their combined "length" is less than 600 (i.e.
         *sqrt(offset_x^2+offset_y^2+offset_Z^2) < 600*). Prior to Copter
         3.2.1 the recommendation was that the absolute value of each offset
         be less than 150 (i.e. *-150 < offset < 150*).

The video below is from earlier versions of the calibration routine but
may still produce good offsets.

..  youtube:: DmsueBS0J3E
    :width: 100%

[site wiki="copter"]
.. _common-compass-setup-advanced_compassmot_compensation_for_interference_from_the_power_wires_escs_and_motors:

CompassMot — compensation for interference from the power wires, ESCs and motors
================================================================================

This is recommended for vehicles that have only an internal compass and
on vehicles where there is significant interference on the compass from
the motors, power wires, etc. CompassMot only works well if you have a
:ref:`battery current monitor <common-powermodule-landingpage>`
because the magnetic interference is linear with current drawn.  It is
technically possible to set-up CompassMot using throttle but this is not
recommended.

Please follow these instructions:

-  Enable the current monitor (aka :ref:`Power Module <common-powermodule-landingpage>`)
-  Disconnect your props, flip them over and rotate them one position
   around the frame.  In this configuration they should push the copter
   down into the ground when the throttle is raised
-  Secure the copter (perhaps with tape) so that it does not move
-  Turn on your transmitter and keep throttle at zero
-  Connect your vehicle's LiPo battery
-  Connect your autopilot to your computer with the usb cable
-  **If using AC3.2:**

   -  Open the **Initial Setup \| Optional Hardware \| Compass/Motor
      Calib** screen
   -  Press the **Start** button

      .. image:: ../../../images/CompassCalibration_CompassMot.png
         :target: ../_images/CompassCalibration_CompassMot.png

-  You should hear your ESCs arming beep
-  Raise the throttle slowly to between 50% ~ 75% (the props will spin!)
   for 5 ~ 10 seconds
-  Quickly bring the throttle back down to zero
-  Press the **Finish** button (AC3.2) or press **Enter** (AC3.1.5) to
   complete the calibration
-  Check the % of interference displayed.  If it is less than 30% then
   your compass interference is acceptable and you should see good
   Loiter, RTL and AUTO performance.  If it is 31% ~ 60% then the
   interference is in the "grey zone" where it may be ok (some users are
   fine, some are not).  If it is higher than 60% you should try moving
   your APM/PX further up and away from the sources of interference or
   consider purchasing an external compass (or 
   :ref:`GPS+compass module<common-positioning-landing-page>` (some of these)).

Here is a video of the procedure based on AC3.1.5:

..  youtube:: 0vZoPZjqMI4
    :width: 100%
[/site]

Manual declination
==================

By default the declination is looked up in a compressed table when the
vehicle first achieves GPS lock. This method is accurate to within 1
degree (which should be sufficient) but if you wish to use the
uncompressed declination:

-  Open the `Declination Website <http://www.magnetic-declination.com/>`__.
-  It should automatically figure out your location based on you IP
   address or you can enter your location

   .. image:: ../../../images/declination.png
       :target: ../_images/declination.png
    
-  Uncheck the **Obtain declination automatically** checkbox and
   manually enter the declination (highlighted in red in the image
   above) into the mission planner's declination field. In this example,
   we would enter "14" Degrees and "13" Minutes.
-  As soon as your cursor exits the field (i.e by pressing Tab) the
   value will be converted to decimal radians and saved to the
   ``COMPASS_DEC`` parameter.

Tuning declination in-flight
============================

Although we do not believe this is ever necessary, you can manually tune
the declination in flight using the Channel 6 tuning knob on your
transmitter by following these steps:

#. Connect your Pixhawk (or other board) to the Mission Planner
#. Go to the **Software \| Copter Pids** screen
#. Set the Ch6 Opt to "Declination", Min to "0.0" and Max to "3.0". 
   This will give a tunable range of -30 to +30 degrees.  Set Max to
   "2.0" to tune from -20 to +20 degrees, etc.

   .. image:: ../../../images/CompassCalibration_TuneDec.png
       :target: ../_images/CompassCalibration_TuneDec.png
    
#. Check the declination is updating correctly when turning the channel
   6 tuning knob to it's maximum position, go to **Config/Tuning \|
   Standard Params** screen, press the **Refresh Params** button and
   ensuring that ``COMPASS_DEC`` is 0.523 (this is 30 degrees expressed
   in radians)

   .. image:: ../../../images/CompassCalibration_TuneDecCheck.png
       :target: ../_images/CompassCalibration_TuneDecCheck.png

#. Fly your copter in Loiter mode in at least two directions and ensure
   that after a fast forward flight you do not see any circling (also
   known as "toilet bowling").
#. If you find it's impossible to tune away the circling then it's
   likely you will require an external compass
   or :ref:`GPS+compass module<common-positioning-landing-page>` (some of these)
   
.. _automatic-compass-offset-calibration:

Automatic Offset Calibration
============================

In the 4.0 releases of ArduPilot, an automatic offset learning feature is available. The :ref:`COMPASS_LEARN<COMPASS_LEARN>` parameter determines how this feature works.

- If set to 3, the offsets will be learned automatically during flight, be saved, and this parameter reset to 0. Position control modes (Loiter, Auto, etc.) should not be used while the offsets are being learned.

.. note:: Setting :ref:`COMPASS_LEARN<COMPASS_LEARN>` to 1 or 2 is not recommended. These modes are deprecated and are either non-functional, or still in development.

  The procedure for :ref:`COMPASS_LEARN<COMPASS_LEARN>` = 3 is:

  1. set :ref:`COMPASS_LEARN<COMPASS_LEARN>` = 3. The message “CompassLearn: Initialised” will appear on the MP’s message tab (it does not appear in red letters on the HUD).
  2. “Bad Compass” will appear but this is nothing to be worried about. We will hopefully make this disappear before the final release.
  3. Arm and drive/fly the vehicle around in whatever mode you like, do some turns “CompassLearn: have earth field” should appear on MP’s message tab and then eventually “CompassLearn: finished”.
  4. If you want you can check the :ref:`COMPASS_LEARN<COMPASS_LEARN>` parameter has been set back to zero (you may need to refresh parameters to see this) and the COMPASS_OFS_X/Y/Z values will have changed.
  5. This method can also be evoked using the RCxOPTION for "Compass Learn". It will activate when the channel goes above 1800uS and automatically complete and save.

.. note: These methods do not fully calibrate the compass, like Onboard Calibration does, setting the scales and (in 4.0 vehicle releases) automatically determining the compass orientation.

Compass error messages
======================

-  **Compass Health**: The compass has not sent a signal for at least
   half a second.
-  **Compass Variance**: In the EKF solution, compass heading disagrees
   with the heading estimate from other inertial sensors. Clicking the
   EKF button on the Mission Planner HUD will show the magnitude of the
   error.
-  **Compass Not Calibrated**: The compass needs to be calibrated.
-  **Compass Offsets High**: One of your compass offsets exceeds 600,
   indicating likely magnetic interference. Check for sources of
   interference and try calibrating again.
