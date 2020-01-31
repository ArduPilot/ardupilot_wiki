.. _autotrim:

========
AutoTrim
========

Wind of course has a strong effect on your copter and will push it around. However you may also find that while flying in stabilize mode, even in a windless environment your copter tends to always drift away in the same direction. This can largely be corrected using the "Save Trim" or "Automatic Trim" functions.

.. note::

    For most users this procedure is not necessary because the :ref:`accelerometer calibration <common-accelerometer-calibration>` does a good job of setting the trim values.

Save Trim
~~~~~~~~~

Save trim is the simpler method and involves essentially transferring your radio transmitter's trims to the autopilot (`video demo here <https://www.youtube.com/watch?v=ayA0uYOqKX4>`__).

1. Check that your CH7 switch goes above 1800 on the MissionPlanner's Hardware > Mandatory Hardware > Radio Calibration screen

.. image:: ../images/MP_SaveTrim_Ch7PWMCheck.png
    :target: ../_images/MP_SaveTrim_Ch7PWMCheck.png

2. Set the CH7 Option to Save Trim on the Software > Copter Pids screen and press the "Write Params" button

.. image:: ../images/MP_SaveTrim_Ch7.png
    :target: ../_images/MP_SaveTrim_Ch7.png

3. With your CH7 switch in the off position, fly your copter in Stabilize mode and use your transmitters's roll and pitch trim to get it flying level

4. Land and put your throttle to zero

5. Release the roll and pitch sticks and switch the CH7 switch high for at least 1 second.  The words "Trim saved" should appear in the MP's Flight Data screen's Messages tab

6. Reset your transmitters roll and pitch trims back to the center and fly again and it should fly level now. If it does not repeat steps 3, 4 & 5

Auto Trim
~~~~~~~~~

With auto trim the roll and pitch trim are captured as you fly in a stable hover.

1. Find a wind free environment with sufficient space to fly your copter without crashing into something

2. Put the vehicle in Stabilize or AltHold mode

3. Hold throttle down and rudder right for 15seconds or until you see the small red, blue and yellow leds flash in a cyclic pattern

4. Fly your copter for about 25 seconds in a stable hover

5. Land and put your throttle to zero and wait for a few seconds (the trims parameters are being saved)

6. Take off again in stabilize mode and check if your copter is flying level now. If not repeat steps 2, 3 and 4

.. note::

    You can also test these procedures above are operational on the ground with your battery disconnected. Connect your autopilot to the mission planner and observing the Flight Data screen as you simulate completing the steps above.

.. image:: ../images/MP_SaveTrim_FlightDataScreen.jpg
    :target: ../_images/MP_SaveTrim_FlightDataScreen.jpg

.. note::

    You can manually set the trim by modifying the :ref:`AHRS_TRIM_X <AHRS_TRIM_X>` and :ref:`AHRS_TRIM_Y <AHRS_TRIM_Y>`. Roll trim is :ref:`AHRS_TRIM_X <AHRS_TRIM_X>`, Pitch trim is :ref:`AHRS_TRIM_Y <AHRS_TRIM_Y>`. Both values are in radians with left roll and forward pitch being negative numbers.

.. note::

    It is nearly impossible to get rid of all drift so that your copter remains completely motionless without any input.

Video demonstrations of Save Trim and Auto Trim
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

..  youtube:: 5z0zuwicZh8
    :width: 100%

..  youtube:: e3OF9ih50PU
    :width: 100%

Desktop method
~~~~~~~~~~~~~~

The trim can also be updated by setting the vehicle level, connecting to
the Mission Planner (or perhaps other ground stations) and selecting
Initial Setup, Mandatory Hardware, Accel Calibration and pushing the
lower "Calibrate Level" button.

.. image:: ../images/AccelCalibration_MP.png
    :target: ../_images/AccelCalibration_MP.png

Please note though that making the HUD level while the vehicle is on the
ground does not necessarily mean it won't drift horizontally while
flying because of other small frame issues including the flight
controller not being perfectly level on the frame and slightly tilted
motors.

.. |MP_SaveTrim_Ch7| image:: ../images/MP_SaveTrim_Ch7.png
    :target: ../_images/MP_SaveTrim_Ch7.png
