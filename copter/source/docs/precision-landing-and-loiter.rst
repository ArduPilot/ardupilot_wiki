.. _precision-landing-and-loiter:

============================
Precision Landing and Loiter
============================

Copter supports **Precision Landing** and **Precision Loiter**, which use an external position reference (aka "landing target") to achieve centimeter-level accuracy during landing or while hovering over a target.

Precision Landing is supported via MAVLink `LANDING_TARGET <https://mavlink.io/en/messages/common.html#LANDING_TARGET>`__ messages
sent from a companion computer such as the `Landmark system <https://landmarklanding.com/products/landmark-precision-landing-system>`__,
or using the `IR-LOCK sensor <https://irlock.com/products/ir-lock-sensor-precision-landing-kit>`__, `beacon <https://irlock.com/products/markone-beacon-v3-0-beta>`__,
and a :ref:`rangefinder <common-rangefinder-landingpage>`.

When the vehicle enters :ref:`Land mode <land-mode>` and a landing target is detected, the target's position will be used to guide the final descent.
Precision Landing can also be performed in :ref:`RTL mode <rtl-mode>` as the final stage of the return-to-launch sequence.

Precision Loiter requires minimal additional setup and is activated by entering :ref:`Loiter mode <loiter-mode>` mode and invoking the Precision Loiter :ref:`Auxiliary Function <common-auxiliary-functions>`.

..  youtube:: _NVCSL9VBvw
    :width: 100%

*This video demonstrates Precision Loiter followed by Precision Landing using the Landmark Precision Landing System.*


Available Systems
=================

.. toctree::
    :maxdepth: 1

    IR-LOCK Sensor & Beacon <precision-landing-irlock>
    Landmark Precision Landing System <precision-landing-landmark>


Quick Start
===========

#. Set :ref:`PLND_ENABLED <PLND_ENABLED>` = **1** and reboot to expose parameters.
#. Select the landing target position source via :ref:`PLND_TYPE <PLND_TYPE>` (e.g., **1** = MAVLink `LANDING_TARGET <https://mavlink.io/en/messages/common.html#LANDING_TARGET>`__, **2** = IR-LOCK, **3** = Gazebo Sim, **4** = SITL; **0** disables).
#. If the landing target camera/sensor is not pointing down or the x-axis is not pointing forward, configure the mounting orientation using :ref:`PLND_ORIENT <PLND_ORIENT>` and :ref:`PLND_YAW_ALIGN <PLND_YAW_ALIGN>`.
#. Verify the target is detected while disarmed and the reported offsets change as you move the target/vehicle.
#. Take off and hover above the target.
#. (Recommended) Confirm stable tracking in Precision Loiter before attempting Precision Landing.
#. Activate :ref:`Land mode <land-mode>` to initiate Precision Landing.

.. note::
   Precision Landing requires a valid horizontal position estimate and a steady attitude solution. Poor GPS/EKF or a noisy rangefinder will degrade performance.


Operation
=========

With Precision Landing enabled and Land mode activated, the external landing target measurements override the target provided by the GPS solution for the final approach.

If the landing target is not found, the vehicle will continue descending vertically until the height specified by :ref:`PLND_ALT_MAX <PLND_ALT_MAX>`.

If the landing target is lost, the behaviour depends on :ref:`PLND_STRICT <PLND_STRICT>`:

- **0** – continue landing using standard Land mode logic (not strict).
- **1** – perform landing retries until :ref:`PLND_RET_MAX <PLND_RET_MAX>` is reached, then land normally.
- **2** – perform landing retries until :ref:`PLND_RET_MAX <PLND_RET_MAX>` is reached, then hover instead of landing.

When a landing retry is triggered, the vehicle will climb to a height/position determined by :ref:`PLND_RET_BEHAVE <PLND_RET_BEHAVE>`.
It will then descend again in hopes of detecting the landing target.
It will keep doing this until :ref:`PLND_RET_MAX <PLND_RET_MAX>` retries have occurred.
Then, depending on :ref:`PLND_STRICT <PLND_STRICT>`, the vehicle will either just land, or it will stay in the air hovering
(useful for, say, landing on boats where you don’t want to land on water).

If the landing target is lost below the height specified by :ref:`PLND_ALT_MIN <PLND_ALT_MIN>`, the vehicle will continue landing vertically.

ArduPilot's EKF assumes the landing target is stationary. Set :ref:`PLND_OPTIONS <PLND_OPTIONS>` bit 0 if landing on a moving target.

Repositioning manually by the pilot during the landing will abort the landing unless :ref:`PLND_OPTIONS <PLND_OPTIONS>` bit 1 (Allow Precision Landing after manual reposition)is set.

Final landing speed may be reduced below :ref:`LAND_SPEED <LAND_SPEED>` as necessary to assure a precise touchdown.
This can be disabled for a faster final land speed by setting :ref:`PLND_OPTIONS <PLND_OPTIONS>` bit 2.

..  youtube:: plM5BJY34Bc
    :width: 100%

*This video demonstrates the behaviour of different PLND_STRICT settings.*


Parameters
==========

- :ref:`PLND_ENABLED <PLND_ENABLED>`: Set to 1 to enable Precision Landing. Refresh parameters to see following:
- :ref:`PLND_TYPE <PLND_TYPE>`: Sets the type of landing target position source.
- :ref:`PLND_ORIENT <PLND_ORIENT>` / :ref:`PLND_YAW_ALIGN <PLND_YAW_ALIGN>`: Sets the mounting orientation of the landing target camera/sensor.
- :ref:`PLND_LAND_OFS_X <PLND_LAND_OFS_X>` / :ref:`PLND_LAND_OFS_Y <PLND_LAND_OFS_Y>`: Sets the desired landing position of the camera/sensor relative to the target. See :ref:`common-sensor-offset-compensation` page for more information.
- :ref:`PLND_CAM_POS_X <PLND_CAM_POS_X>` / :ref:`PLND_CAM_POS_Y <PLND_CAM_POS_Y>` / :ref:`PLND_CAM_POS_Z <PLND_CAM_POS_Z>`: Sets the camera/sensor position relative to the vehicle body frame.
- :ref:`PLND_XY_DIST_MAX <PLND_XY_DIST_MAX>`: Even if the landing target is detected, the vehicle will not start descending if it is further than this many meters away. Set to 0 to always descend.
- :ref:`PLND_STRICT <PLND_STRICT>`: Controls the landing behaviour if the target is lost.
- :ref:`PLND_ALT_MIN <PLND_ALT_MIN>`: If the target is lost below this height, the vehicle will continue descending vertically. Set to 0 to disable this.
- :ref:`PLND_ALT_MAX <PLND_ALT_MAX>`: If the target is not detected, the vehicle will continue descending vertically until this height. Below this height, if the target is still not found, the landing retry logic set by :ref:`PLND_STRICT <PLND_STRICT>` will be applied. Set to 0 to disable this.
- :ref:`PLND_OPTIONS <PLND_OPTIONS>`: Set bit 0 if landing target is moving, set bit 1 to allow the precision landing to continue after a manual reposition, and set bit 2 to enable a faster final descent.

Additional ``PLND_`` parameters are listed in the `Complete Parameter List <https://ardupilot.org/copter/docs/parameters.html#plnd-parameters>`__.


Precision Loiter
================

Precision Loiter uses the same backend and parameters as Precision Landing.

To enable Precision Loiter, set an :ref:`Auxiliary Function Switch <common-auxiliary-functions>` to 39 (PrecLoiter Enable).

Entering Loiter mode and activating the switch will cause the vehicle to hold its position above the target.


MAVLink LANDING_TARGET Message Based Systems
============================================

Using a companion computer, camera, and fiducial marker system such as `AprilTag <https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html>`__,
the position differences to the landing target location can be sent to the autopilot using MAVLink `LANDING_TARGET <https://mavlink.io/en/messages/common.html#LANDING_TARGET>`__ messages.
Here is an example system description: `Precision Landing with Realsense T265 Camera and AprilTag <https://discuss.ardupilot.org/t/precision-landing-with-realsense-t265-camera-and-apriltag-part-1-2/48978>`__
