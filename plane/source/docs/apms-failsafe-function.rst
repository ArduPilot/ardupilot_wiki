.. _apms-failsafe-function:

=======================
Plane Failsafe Function
=======================

Plane has a limited failsafe function which is designed to do three
things:

#. Detect complete loss of RC signal (if the RC receiver is able to
   generate a predictable signal-loss behavior) and initiate a defined
   auto-mode response, such as returning to home. Some RC equipment can
   do this, and some can't (see below for details on how to use it if
   yours supports this function).
#. Detect loss of telemetry for more than FS_LONG_TIMEOUT sec and switch to return to
   launch (RTL) mode (GCS Failsafe).
#. Detect loss of GPS for more than 20 seconds and switch into Dead
   Reckoning mode until GPS signal is regained.

Here's what the failsafe **will not do**:

#. Detect if one more more individual RC channel has failed or become disconnected
#. Detect if you're flying too far away or are about to hit the ground
#. Detect autopilot hardware failures, such as low-power brownouts or in-air reboots
#. Detect if the Plane software is not operating correctly
#. Detect other problems with the aircraft, such as motor failures or
   low battery situations (although the latter can be set up through the
   main code if you have the right voltage/current sensor)
#. Otherwise stop you from making setup or flight mistakes

Plane Failsafe Documentation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. _apms-failsafe-function_throttle_failsafe:

Throttle Failsafe
~~~~~~~~~~~~~~~~~

**How it works.** Your RC transmitter outputs a PWM signal that is
captured by your receiver and relayed to the autopilot. Each channel on
your transmitter has a PWM range usually between 1100 - 1900 with 1500
being its neutral position. When you start your radio calibration on the
mission planner, all your values will be at 1500. By moving your sticks,
knobs and switches you will set your PWM range for each channel. The
autopilot monitors your throttle channel and if it notices a drop lower
than THR_FS_VALUE (Default is 950) it will go into failsafe mode.

RC transmitters usually have a default range for each channel that goes
from -100% to 100%, however most transmitters will allow you to extend
this to -150% and 150% respectively. In the default setup, bringing your
throttle to -100% will translate to a value close to 1100 and bringing
it to -150% will translate to a value closer to 900. What we want to
achieve is to let your receiver know that the throttle can go as low as
-150% but keep the autopilot control range between -100% and 100%.
Meaning that when flying, our throttle values will range between 1100 -
1900.

-  If we lose RC communication, the receiver if set up properly, will
   drop to the lowest known throttle value of ~900. This value falls
   bellow the THR_FS_VALUE and will trigger the autopilot to go into
   failsafe mode.
-  First the autopilot will go into short failsafe (FS_SHORT_ACTN,
   0=Disabled, 1=Enabled) when it detects loss of signal for more than
   FS_SHORT_TIMEOUT sec. The default setting for short failsafe is Circle mode.
-  If the RC signal is regained during the short failsafe, the flight
   will return to auto mode.
-  If the loss of signal is longer than FS_LONG_TIMEOUT sec the autopilot will go
   into long failsafe (FS_LONG_ACTN, 0=Disabled, 1=Enabled).
-  The default setting for long failsafe is RTL (Return to Launch).
-  Once the long failsafe (RTL mode) has been entered at the conclusion
   of the short failsafe the RTL mode will continue even if your RC
   signal is reacquired.


::

             Ext. Range       Normal Range       Ext. Range
        |-----------------|-----------------|-----------------|
      -150%             -100%              100%              150%

        |_________________|
                 |
              Failsafe

**Setup.**

#. Enable throttle failsafe by setting THR_FS_Value to 1 (0=Disabled,
   1=Enabled).
#. First turn on your transmitter and enable the throttle range to
   extend past -100%, we want to extend the throttle range past its low
   threshold.
#. Once this is done, bind with your receiver. This will let your
   receiver know the lowest possible value for your throttle channel.
#. Next revert the first change you made to the transmitter to limit the
   throttle to the original range.
#. Do the radio calibration using the Mission Planner.
#. Once the radio calibration is completed, drop the throttle on your
   transmitter and read what PWM value is being output to the mission
   planner on that channel.
#. Turn off the transmitter. You should see the value drop
   significantly. This will be the PWM value relayed to the autopilot in
   the event RC link was lost during flight.
#. Make sure THR_FS_VALUE is an adequate number to trigger the
   failsafe function on the autopilot.
#. Make sure FS_SHORT_ACTN and FS_LONG_ACTN are both enabled (set to
   1).
#. Connect on the mission planner with your RC transmitter on. Verify on
   the bottom right corner of the HUD that you are “flying” in a non
   auto mode (Manual, Stabilize, FBW are ok).
#. Turn off your transmitter. After S_SHORT_TIMEOUT sec the flight mode should
   switch to Circle. After FS_LONG_TIMEOUT sec the flight mode should switch to RTL.
   If you observe this behavior, your failsafe function has been set up
   correctly.

**Transmitter Tutorials:**

`Spektrum Setup <http://diydrones.com/profiles/blogs/spektrum-dx8-and-ar8000-failsafe-setup>`__

GCS Failsafe
~~~~~~~~~~~~

**How it works.** When flying while using telemetry on the GCS, the
autopilot can be programmed to trigger into failsafe mode if it loses
telemetry. In the event that the autopilot stops receiving MAVlink
(telemetry protocol) heartbeat messages for more than FS_LONG_TIMEOUT sec, the GCS
failsafe (FS_GCS_ENABL, 0=Disabled, 1=Enabled) will trigger the
autopilot to go into long failsafe and change the flight mode to RTL.

**Setup.**

#. Set FS_GCS_ENABL to 1 to enable it.
#. Connect to the Mission Planner via telemetry. Verify on the bottom
   right corner of the HUD that you are “flying” in a non auto mode
   (Manual, Stabilize, FBW are ok).
#. Unplug one of the telemetry radios. After a few minutes power off
   your autopilot. (Remember the autopilot will not go into failsafe
   until FS_LONG_TIMEOUT seconds of MAVlink inactivity have passed).
#. Connect your autopilot to the mission planner and pull the logs.
   Verify on the log that the autopilot went into RTL after FS_LONG_TIMEOUT sec of
   MAVlink inactivity.

Failsafe Parameters and their meanings
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Short failsafe action (Plane:FS_SHORT_ACTN)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The action to take on a short (FS_SHORT_TIMEOUT seconds) failsafe event in AUTO,
GUIDED or LOITER modes. A short failsafe event in stabilization modes
will always cause a change to CIRCLE mode. In AUTO mode you can choose
whether it will RTL (ReturnToLaunch) or continue with the mission. If
FS_SHORT_ACTN is 0 then it will continue with the mission, if it is 1
then it will enter CIRCLE mode, and then enter RTL if the failsafe
condition persists for FS_LONG_TIMEOUT seconds.

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>VALUE</th>
   <th>MEANING</th>
   </tr>
   <tr>
   <td>0</td>
   <td>Continue</td>
   </tr>
   <tr>
   <td>1</td>
   <td>Circle/ReturnToLaunch</td>
   </tr>
   </tbody>
   </table>

Long failsafe action (Plane:FS_LONG_ACTN)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The action to take on a long (FS_LONG_TIMEOUT second) failsafe event in AUTO, GUIDED
or LOITER modes. A long failsafe event in stabilization modes will
always cause an RTL (ReturnToLaunch). In AUTO modes you can choose
whether it will RTL or continue with the mission. If FS_LONG_ACTN is 0
then it will continue with the mission, if it is 1 then it will enter
RTL mode. Note that if FS_SHORT_ACTN is 1, then the aircraft will
enter CIRCLE mode after FS_SHORT_TIMEOUT seconds of failsafe, and will always enter
RTL after FS_LONG_TIMEOUT seconds of failsafe, regardless of the FS_LONG_ACTN
setting.

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>VALUE</th>
   <th>MEANING</th>
   </tr>
   <tr>
   <td>0</td>
   <td>Continue</td>
   </tr>
   <tr>
   <td>1</td>
   <td>ReturnToLaunch</td>
   </tr>
   </tbody>
   </table>

Failsafe battery voltage (Plane:FS_BATT_VOLTAGE)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Battery voltage to trigger failsafe. Set to 0 to disable battery voltage
failsafe. If the battery voltage drops below this voltage then the plane
will RTL

-  Units: Volts

Failsafe battery milliAmpHours (Plane:FS_BATT_MAH)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Battery capacity remaining to trigger failsafe. Set to 0 to disable
battery remaining failsafe. If the battery remaining drops below this
level then the plane will RTL

-  Units: mAh

GCS failsafe enable (Plane:FS_GCS_ENABL)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Enable ground control station telemetry failsafe. Failsafe will trigger
after FS_SHORT_TIMEOUT and / or FS_LONG_TIMEOUT seconds of no MAVLink heartbeat messages. WARNING: Enabling
this option opens up the possibility of your plane going into failsafe
mode and running the motor on the ground it it loses contact with your
ground station. If this option is enabled on an electric plane then
either use a separate motor arming switch or remove the propeller in any
ground testing.

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>VALUE</th>
   <th>MEANING</th>
   </tr>
   <tr>
   <td>0</td>
   <td>Disabled</td>
   </tr>
   <tr>
   <td>1</td>
   <td>Enabled</td>
   </tr>
   </tbody>
   </table>
