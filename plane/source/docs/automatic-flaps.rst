.. _automatic-flaps:

===============
Automatic Flaps
===============

Plane can control flaps in autonomous modes based on an airspeed
schedule. Simply put you can specify two speeds and two flap settings.
If your target speed (not your actual speed) is above the specified
speeds then your flaps are set to the default (trim) position. If your
target speed is lowered below the first flap speed, then flaps are
deployed to the first position, and if your target speed is lowered
below the second flap speed, then flaps are deployed to the second
position.

The target speed can be commanded by changing the value of cruise_speed
in the parameter interface, by using the Do_Set_Speed command in a
mission, or by the throttle stick position in FBW-B.

Flaps can be configured on any channel (in older versions flaps could only be configured on channel 5, 6, 7 or 8 with 8 not available for manual control).

Software configuration
~~~~~~~~~~~~~~~~~~~~~~

The first step in setting up flaps is to set the parameter for the
channel function for the channel you are using for flaps to a value of
3. For example, if you have flaps on channel 5 then set :ref:`SERVO5_FUNCTION<SERVO5_FUNCTION>` to
3.

Next set the parameters for your two flap speeds and flap values. These
parameters are FLAP_1\_PERCNT, FLAP_1\_SPEED, FLAP_2\_PERCNT,
FLAP_2\_SPEED.

Redo your radio calibration. Be sure to set the flap switch to the zero
flap position before finishing the radio calibration so that zero flaps
is stored as the trim (default) position.

That is basically all the setup required for flaps. If you find that
your flaps are moving backwards from what you expect, change the
reversing parameter for your flap channel, for example
:ref:`SERVO5_REVERSED<SERVO5_REVERSED>` . Set it to 1 to reverse the flaps, set to 0 for normal
operation.

Using flaps
~~~~~~~~~~~

In Manual, Stabilize, or FBW-A modes your flaps will operate manually
and you can set them with your transmitter.

In FBW-B the target airspeed is set by the position of the throttle
stick. If set up properly you should see flaps deploy as you lower the
throttle stick through the two flap airspeed ranges.

In RTL, Guided, and Auto modes flaps settings are determined by the
schedule in your parameters and the current value of cruise_speed. You
can change cruise_speed to a value in one of the flap ranges from the
GCS and should see the flaps deploy. Also, if you use the
Do_Change_Speed command in mission flaps will be deployed when the
target speed is changed into one of the flap speed ranges.

Using Flaperons
===============

You can also setup flaperons. Please see the separate :ref:`flaperon guide <flaperons-on-plane>` for details.
