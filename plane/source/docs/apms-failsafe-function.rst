.. _apms-failsafe-function:

=======================
Plane Failsafe Function
=======================

Plane has a limited failsafe function which is designed to do four
things:

#. Detect a complete loss of RC signal and initiate a defined response, such as returning to home. Detection is either by lack of data/pulses from the receiver, the throttle channel PWM value falling below a certain point set by :ref:`THR_FS_VALUE<THR_FS_VALUE>`, or the receiver sets a FS bit in its data stream for those protocols supporting this. Detection of these must be enabled by setting :ref:`THR_FAILSAFE<THR_FAILSAFE>` = 1 or 2.
#. Optionally, detect loss of telemetry (GCS Failsafe) and take an programmable action, such as switching to return to launch (RTL) mode.
#. Detect loss of GPS for more than 20 seconds and switch into Dead Reckoning mode until GPS signal is regained. See https://youtu.be/0VMx2u8MlUU for a demo.
#. Optionally, detect low battery conditions (voltage/remaining capacity) and initiate a programmable response, such as returning to home. ArduPilot supports this on multiple batteries.

Here's what the failsafe **will not do**:

#. Detect if one or more individual RC channel has failed or become disconnected
#. Detect if you're flying too far away or are about to hit the ground
#. Detect autopilot hardware failures, such as low-power brownouts or in-air reboots
#. Detect if the Plane software is not operating correctly
#. Detect other problems with the aircraft, such as motor failures 
#. Otherwise stop you from making setup or flight mistakes


.. note:: See :ref:`advanced-failsafe-configuration` for extended failsafe configurations.


Plane Failsafe Documentation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. _apms-failsafe-function_throttle_failsafe:

Throttle Failsafe
~~~~~~~~~~~~~~~~~

**How it works.** Your RC transmitter outputs a PWM signal that is
captured by your receiver and relayed to the autopilot, either as a pulse width or data in a serial data stream. Each channel on
your transmitter has a PWM range usually between 1100 - 1900 with 1500
being its neutral position. When you start your radio calibration on the
mission planner, all your values will be at 1500. By moving your sticks,
knobs and switches you will set your PWM range for each channel. The
autopilot monitors your throttle channel and if it notices a drop lower
than :ref:`THR_FS_VALUE<THR_FS_VALUE>` (Default is 950) it will go into Throttle Failsafe mode.

.. note: ArduPilot can also detect if the RC Receiver becomes disconnected or dead (no PWM pulses), if the PWM values are grossly out of range (RC Receiver failure), or if the failsafe bit in an receiver's data stream is set, and will initiate a Failsafe.

RC transmitters usually have a default range for each channel that goes
from -100% to 100%, however most transmitters will allow you to extend
this to -150% and 150% respectively. In the default setup, bringing your
throttle to -100% will translate to a value close to 1100 and bringing
it to -150% will translate to a value closer to 900. What we want to
achieve is to let your receiver know that the throttle can go as low as
-150% but keep the autopilot control range between -100% and 100%.
Meaning that when flying, our throttle values will range between 1100 -
1900.

-  If we lose RC communication, and the receiver is capable and set up properly, it will
   drop to the lowest known throttle value of ~900. This value falls
   bellow the :ref:`THR_FS_VALUE<THR_FS_VALUE>` and will trigger the autopilot to go into
   an RC failsafe mode, know as Throttle Failsafe. Alternatively, if communication from the receiver is lost, either by its sending no pulses during signal loss, or by wiring disconnection, it will also go into RC failsafe mode . Finally, if the :ref:`RC_OPTIONS<RC_OPTIONS>` bit 2 is not set, and the receiver protocol has a failsafe bit, it can trigger the RC failsafe. Receiver protocols which have this bit are: Sbus, SXRL, SXRL2, SumD, FPort, FPort2, and iBUS.
-  When failsafe is entered, all RC inputs (except throttle in the case of throttle initiated failsafe), are ignored as the autopilot takes its failsafe actions.
-  First, the autopilot will go into short failsafe (:ref:`FS_SHORT_ACTN<FS_SHORT_ACTN>` ),
   when it detects loss of signal for more than :ref:`FS_SHORT_TIMEOUT<FS_SHORT_TIMEOUT>` sec. The default setting for short failsafe is Circle mode.
-  If the RC signal is regained during the short failsafe, the flight
   will return to the previous mode.
-  If the loss of signal is longer than :ref:`FS_LONG_TIMEOUT<FS_LONG_TIMEOUT>` sec the autopilot will go into long failsafe :ref:`FS_LONG_ACTN<FS_LONG_ACTN>` .
-  The default setting for long failsafe is RTL (Return to Launch).

.. note:: Once the long failsafe has been entered at the conclusion
   of the short failsafe the :ref:`FS_LONG_ACTN<FS_LONG_ACTN>`  mode will continue even if your RC
   signal is reacquired. Once reacquired, the mode can only be exited via a mode change. In addition, other failsafes, such as battery failsafe, can also change the mode, if they occur subsequently.

::

             Ext. Range       Normal Range       Ext. Range
        |-----------------|-----------------|-----------------|
      -150%             -100%              100%              150%

        |_________________|
                 |
              Failsafe

**Setup.**

#. Enable RC failsafe by setting :ref:`THR_FAILSAFE<THR_FAILSAFE>` to 1 (0=Disabled,
   1=Enabled, 2=Enabled, No Failsafe action undertaken).
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
#. Make sure :ref:`THR_FS_VALUE<THR_FS_VALUE>` is an adequate number to trigger the
   failsafe function on the autopilot.
#. Make sure :ref:`FS_SHORT_ACTN<FS_SHORT_ACTN>` or :ref:`FS_LONG_ACTN<FS_LONG_ACTN>` , or both are enabled (set to a non-zero value).
#. Connect on the mission planner with your RC transmitter on. Verify on
   the bottom right corner of the HUD that you are “flying” in a non
   auto mode (Manual, Stabilize, FBW are ok).
#. Turn off your transmitter. After :ref:`FS_SHORT_TIMEOUT<FS_SHORT_TIMEOUT>` sec , if enabled, the flight mode should
   switch to :ref:`FS_SHORT_ACTN<FS_SHORT_ACTN>`. After :ref:`FS_LONG_TIMEOUT<FS_LONG_TIMEOUT>` sec, if enabled, the flight mode should switch to :ref:`FS_LONG_ACTN<FS_LONG_ACTN>`.
   If you observe this behavior, your failsafe function has been set up
   correctly.


**Transmitter Tutorials:**

`Spektrum Setup <https://diydrones.com/profiles/blogs/spektrum-dx8-and-ar8000-failsafe-setup>`__


GCS Failsafe
~~~~~~~~~~~~

**How it works.** When flying while using telemetry on the GCS, the
autopilot can be programmed to trigger into failsafe mode if it loses
telemetry. In the event that the autopilot stops receiving MAVlink
(telemetry protocol) heartbeat messages. :ref:`FS_SHORT_ACTN<FS_SHORT_ACTN>` and :ref:`FS_LONG_ACTN<FS_LONG_ACTN>` apply just in the case of a Throttle Failsafe.

**Setup.**

#. Set :ref:`FS_GCS_ENABL<FS_GCS_ENABL>` to 1 to enable it.
#. Connect to the Mission Planner via telemetry. Verify on the bottom
   right corner of the HUD that you are “flying” in a non auto mode
   (Manual, Stabilize, FBW are ok).
#. Unplug one of the telemetry radios. After a few minutes power off
   your autopilot. (Remember the autopilot will not go fully into failsafe
   until :ref:`FS_LONG_TIMEOUT<FS_LONG_TIMEOUT>` seconds of MAVLink inactivity have passed).
#. Connect your autopilot to the mission planner and pull the logs.
   Verify on the log that the autopilot went into RTL after :ref:`FS_LONG_TIMEOUT<FS_LONG_TIMEOUT>` sec of MAVLink inactivity.

Configuring for Ground Control Station Control beyond RC range
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If the telemetry range exceeds the RC transmitter range, then it may be desired to prevent loss of RC signal from initiating a failsafe. Reliance on the above GCS failsafe would be then be used to provide failsafe protection. In order to prevent the RC system from interfering with GCS operation, set :ref:`THR_FAILSAFE<THR_FAILSAFE>` = 2. This prevents the RC failsafe action from being taken, but still detects the failsafe condition and ignores the RC inputs, preventing possible interference to Ground Control Station control of the vehicle. Control via the RC system can be resumed once back into its range.

Configuring for valid RC outputs while in RC Failsafe
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Normally, the RC channels are ignored when in RC Failsafe (except the throttle channel, but for failsafe detection exit only). Sometimes it is desirable to allow the preset signal loss values( for receivers capable of this ), to be used in the event of an RC failsafe. For example, parachute activation, or other controls via RC passthrough (see :ref:`common-auxiliary-functions`) could be desired when in RC failsafe. For receivers with this capability and which use a FS data bit, setting :ref:`RC_OPTIONS<RC_OPTIONS>` bit 2 to "1", can accomplish this. In this case, the FS bit is ignored. Upon RC signal loss the receiver would go to its pre-set channel outputs values, but a failsafe action would not be taken by ArduPilot, since the receiver is still outputting valid data as far as ArduPilot can detect. The fixed RC channel values would processed as normal by ArduPilot.


.. note:: In this setup, it is usually necessary to make sure that the flight mode channel will force an RTL or AUTO mission to return the vehicle when the receiver loses RC signal, since no failsafe action will be taken, otherwise. The values of the flight control channels for Roll, Pitch, Yaw and Throttle need to be appropriately set also (usually neutral positions).

Battery Failsafe
~~~~~~~~~~~~~~~~

.. note::

    This failsafe requires the vehicle have a working :ref:`Power Module <common-powermodule-landingpage>`.

.. note:: ArduPilot firmware versions 4.0 and later support up to 10 batteries/power monitors. All the  discussion below applies to those optional batteries also. Each can trigger a failsafe and each can have different actions and setup values. In addition, a group of batteries can be treated as a single unit, see ``BATTx_MONITOR`` = 10.

When the failsafe will trigger
------------------------------

If enabled and set-up correctly the battery failsafe will trigger if the main battery's

-  voltage drops below the voltage held in the :ref:`BATT_LOW_VOLT <BATT_LOW_VOLT>` parameter (or FS_BATT_VOLTAGE in older versions) for more than 10 seconds. If set to zero (the Plane default value) the voltage based trigger will be disabled.
-  remaining capacity falls below the :ref:`BATT_LOW_MAH <BATT_LOW_MAH>` parameter (or FS_BATT_MAH in older versions) 20% of the battery's full capacity is a good choice (i.e. "1000" for a 5000mAh battery).  If set to zero the capacity based trigger will be disabled (i.e. only voltage will be used)

What will happen
----------------

When the failsafe is triggered:

-  Buzzer will play a loud low-battery alarm
-  LEDs will flash yellow
-  A warning message will be displayed on the ground station's HUD (if telemetry is connected)
-  :ref:`BATT_FS_LOW_ACT<BATT_FS_LOW_ACT>`  will be executed

Two-Stage Battery Failsafe
--------------------------

Plane 3.9 (and higher) includes a two-layer battery failsafe.  This allows setting up a follow-up action if the battery voltage or remaining capacity falls below an even lower threshold.

- :ref:`BATT_CRT_VOLT <BATT_CRT_VOLT>` - holds the secondary (lower) voltage threshold.  Set to zero to disable. Default is zero.
- :ref:`BATT_CRT_MAH <BATT_CRT_MAH>` - holds the secondary (lower) capacity threshold.  Set to zero to disable. Default is zero.
- :ref:`BATT_FS_CRT_ACT <BATT_FS_CRT_ACT>` - holds the secondary action to take.  A reasonable setup would be to have :ref:`BATT_FS_LOW_ACT <BATT_FS_LOW_ACT>` = 2 (RTL) and :ref:`BATT_FS_CRT_ACT <BATT_FS_CRT_ACT>` = 1 (Land)

Advanced Battery Failsafe Settings
----------------------------------

- :ref:`BATT_FS_VOLTSRC <BATT_FS_VOLTSRC>` allows configuring whether the raw battery voltage or a sag corrected voltage is used
- :ref:`BATT_LOW_TIMER <BATT_LOW_TIMER>` can configure how long the voltage must be below the threshold for the failsafe to trigger
- ``BATTx_`` parameters can be setup to trigger the failsafe on other batteries

Failsafe Parameters and their Meanings
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Short failsafe action (:ref:`FS_SHORT_ACTN<FS_SHORT_ACTN>` )
------------------------------------------------------------

The action to take on a short (:ref:`FS_SHORT_TIMEOUT<FS_SHORT_TIMEOUT>` seconds) failsafe event . A short failsafe event in plane stabilization modes can be set to change mode to CIRCLE or FBWA, or be disabled completely. In QuadPlane stabilization modes, it will change to QLAND or QRTL, dependent upon which :ref:`Q_OPTIONS<Q_OPTIONS>` is selected.

In AUTO, LOITER and GUIDED modes you can also choose for it continue with the mission and ignore the short failsafe. If :ref:`FS_SHORT_ACTN<FS_SHORT_ACTN>` is 0 then it will continue with the mission, if it is 1 then it will enter CIRCLE mode.

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
   <tr>
   <td>2</td>
   <td>FBWA</td>
   </tr>
   <tr>
   <td>3</td>
   <td>Disabled</td>
   </tr>
   </tbody>
   </table>

Long failsafe action (:ref:`FS_LONG_ACTN<FS_LONG_ACTN>` )
---------------------------------------------------------

The action to take on a long (:ref:`FS_LONG_TIMEOUT<FS_LONG_TIMEOUT>` seconds) failsafe event. If the aircraft was in a stabilization or manual mode when failsafe started and a long failsafe occurs then it will change to RTL mode if :ref:`FS_LONG_ACTN<FS_LONG_ACTN>` is 0 or 1, and will change to FBWA  and idle the throttle if :ref:`FS_LONG_ACTN<FS_LONG_ACTN>` is set to 2.

If the aircraft was in an auto mode (such as AUTO or GUIDED) when the failsafe started then it will continue in the auto mode if :ref:`FS_LONG_ACTN<FS_LONG_ACTN>` is set to 0, will change to RTL mode if :ref:`FS_LONG_ACTN<FS_LONG_ACTN>` is set to 1 and will change to FBWA mode and idle the throttle if :ref:`FS_LONG_ACTN<FS_LONG_ACTN>` is set to 2. If :ref:`FS_LONG_ACTN<FS_LONG_ACTN>` is set to 3, the parachute will be deployed (make sure the chute is configured and enabled).

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
   <tr>
   <td>2</td>
   <td>FBWA Glide</td>
   </tr>
   <tr>
   <td>3</td>
   <td>Deploy Parachute</td>
   </tr>
   </tbody>
   </table>

In a QuadPlane, if in VTOL operation in modes others than AUTO or GUIDED, the action taken will be either a QRTL or QLAND, depending on the :ref:`Q_RTL_MODE<Q_RTL_MODE>` bit mask setting for bit 5. And if in fixed-wing operation, and the long or short failsafe action is a mode change to RTL, then the :ref:`Q_RTL_MODE<Q_RTL_MODE>` will determine behavior at the end of that RTL, just as in the case of a regular mode change to RTL.

GCS failsafe enable (:ref:`FS_GCS_ENABL<FS_GCS_ENABL>` )
--------------------------------------------------------

Enable ground control station telemetry failsafe. Failsafe will trigger
after :ref:`FS_SHORT_TIMEOUT<FS_SHORT_TIMEOUT>` and/or :ref:`FS_LONG_TIMEOUT<FS_LONG_TIMEOUT>` seconds of no MAVLink heartbeat or RC Override messages.

.. warning:: Enabling this option opens up the possibility of your plane going into failsafe mode and running the motor on the ground if it loses contact with your ground station. While the code attempts to verify that the plane is indeed flying and not on the ground before entering this failsafe, it is safer if this option is enabled on an electric plane, to either use a separate motor arming switch or remove the propeller in any ground testing, if possible.

There are three possible enabled settings. Seeing :ref:`FS_GCS_ENABL<FS_GCS_ENABL>` to 1 means that GCS failsafe will be triggered when the aircraft has not received a MAVLink HEARTBEAT message. Setting :ref:`FS_GCS_ENABL<FS_GCS_ENABL>` to 2 means that GCS failsafe will be triggered on either a loss of HEARTBEAT messages, or a RADIO_STATUS message from a MAVLink enabled telemetry radio indicating that the ground station is not receiving status updates from the aircraft, which is indicated by the RADIO_STATUS.remrssi field being zero (this may happen if you have a one way link due to asymmetric noise on the ground station and aircraft radios).Setting :ref:`FS_GCS_ENABL<FS_GCS_ENABL>` to 3 means that GCS failsafe will be triggered by Heartbeat(like option one), but only in AUTO mode. WARNING: Enabling this option opens up the possibility of your plane going into failsafe mode and running the motor on the ground it it loses contact with your ground station. If this option is enabled on an electric plane then you should enable :ref:`ARMING_REQUIRE<ARMING_REQUIRE>` .

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
   <td>Heartbeat</td>
   </tr>
   <tr>
   <td>2</td>
   <td>Heartbeat and REMRSSI</td>
   </tr>
   <tr>
   <td>3</td>
   <td>Heartbeat and AUTO</td>
   </tr>
   </tbody>
   </table>

Failsafe Diagnosis in Logs or GCS
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

GCSs will often display text indicating the type of failsafe encountered, such as "Failsafe Short event on: type=1/reason=3". Type and Reason can be determined using the table below:


.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>TYPE</th>
   <th>MEANING</th>
   </tr>
   <tr>
   <td>0</td>
   <td>None</td>
   </tr>
   <tr>
   <td>1</td>
   <td>Short Failsafe</td>
   </tr>
   <tr>
   <td>2</td>
   <td>Long Failsafe</td>
   </tr>
   <tr>
   <td>3</td>
   <td>GCS Failsafe</td>
   </tr>
   </tbody>
   </table>
   
.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>REASON</th>
   <th>MEANING</th>
   </tr>
   <tr>
   <td>0</td>
   <td>Unknown</td>
   </tr>
   <tr>
   <td>1</td>
   <td>RC Command</td>
   </tr>
   <tr>
   <td>2</td>
   <td>GCS Command</td>
   </tr>
   <tr>
   <td>3</td>
   <td>Radio Failsafe</td>
   </tr>
   <tr>
   <td>4</td>
   <td>GCS Failsafe</td>
   </tr>
   <tr>
   <td>5</td>
   <td>EKF Failsafe</td>
   </tr>
   <tr>
   <td>6</td>
   <td>GPS Glitch</td>
   </tr>
   <tr>
   <td>9</td>
   <td>Fence Breached</td>
   </tr>
   <tr>
   <td>10</td>
   <td>Terrain</td>
   </tr>
   <tr>
   <td>18</td>
   <td>Crash</td>
   </tr>
   </tbody>
   </table>
   <tr>
   <td>25+</td>
   <td>General unspecific</td>
   </tr>


Independent Watchdog
--------------------

See :ref:`common-watchdog` for details.
