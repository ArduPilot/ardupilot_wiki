.. _example-configuration:

=====================
Example Configuration
=====================

This topic provides additional recommended configuration for Rover
vehicles during initial setup and testing. It assumes that the
configuration is performed using *Mission Planner* (and that *Mission
Planner* is already :ref:`installed <planner:install-mission-planner>` and
:ref:`connected <common-connect-mission-planner-autopilot>`).

#. Go to the **Config/Tuning** tab.
#. Select the **Planner** item in the left menu and ensure the
   **Advanced View** item on the bottom right is checked.
#. Click on the **Standard Params** item in the left menu and install
   some conservative starting parameters.

   -  Set the **Channel 7** Option to **Learn Waypoint** if it is not
      already set to it.
   -  Reduce **Target cruise speed in auto** from 5.00 to **2.50**.
   -  Reduce **Base throttle percentage** from 50 to **30**.

.. warning::

   This step is important as all the navigation in auto modes is based on the cruise speed and throttle configuration. See :ref:`Rover Tuning <tuning-steering-and-navigation-for-a-rover>`

#. Set a failsafe to turn your vehicle off (put it in HOLD mode) if you
   lose RC reception for 5 seconds.

   -  Set **Failsafe Action** to **HOLD**.
   -  Set **GCS failsafe enable** to **Disabled** unless you are using a
      3DR Telemetry radio.
   -  Set **Throttle Failsafe Enable** to **Enabled**
   -  Failsafe is preset to 5 seconds but can be adjusted in the **Full
      Parameter List** with the **FS_TIMEOUT** parameter.

#. Select the **Advanced Params** item from the left menu and ensure the
   **Board Orientation** Item matches your setup.

   -  If your autopilot is mounted upside down (e.g. for improved
      connector access), set the orientation to **Roll 180** (8).
   -  If your autopilot is mounted right side up select **None**.

#. Scroll down to **Compass Orientation** and ensure it is set correctly
   for your compass. This should already have been set in :ref:`Compass Calibration <common-compass-calibration-in-mission-planner>`.
#. Scroll down to **RC trim PWM (RC3_TRIM)** and set it up for (reverse
   - centre - forward) throttle or not.

   -  Throttle is normally channel 3, if you have changed it to another
      channel go there instead.
   -  If you have a centre loaded throttle with reverse, set this value
      to **1500**. If not then leave it at **1100**.

#. The Minimum Throttle parameter (``THR_MIN``) is mostly useful for
   rovers with internal combustion motors, to prevent the motor from
   cutting out in auto mode.
#. The Maximum Throttle parameter (``THR_MAX``) can be used to prevent
   overheating a ESC or motor on an electric rover.
