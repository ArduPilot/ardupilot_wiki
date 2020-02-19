.. _advanced-failsafe-configuration:

===============================
Advanced Failsafe Configuration
===============================

The core failsafe functionality of Plane is based on RTL (Return to
Launch). It is able to initiate a RTL if the aircraft loses contact with
the ground station or loses RC control. That is fine for most users, but
in some situations more advanced failsafe capabilities are needed. That
is what the Advanced Failsafe options are for.

Background
----------

The advanced failsafe options for Plane were added for the `Outback Challenge <http://www.uavoutbackchallenge.com.au/>`__ competition, and
the features are designed to fit the rules of that competition. Whereas
normal failsafe is oriented around saving the aircraft in case control
is lost by the pilot, the OBC rules are oriented around ensuring safe
operation in a defined region of airspace. This means that the advanced
failsafe options are designed to deliberately crash the aircraft if
there is any risk that it may fly outside of the region of airspace
defined by a geographic boundary and a maximum altitude.

While the advanced failsafe features of Plane were designed for the OBC
competition, they can be useful in other situations, and are very
flexible. They are also harder to configure, so please read this page
carefully several times before deciding whether to enable these features
on your aircraft.

Mission based
-------------

The key difference between normal failsafe and the advanced failsafe
(AFS) options is that the AFS options are mission based. When a failsafe
option (such as loss of GPS lock or loss of ground station
communications link) happens, the AFS options specify a waypoint number
in the mission that the aircraft will switch to. This allows the pilot
to configure a complex series of actions to take when a failsafe event
occurs - anything that can be scripted as an Plane mission can be made
to happen on failsafe events.

Typically the pilot will setup the AFS options for failsafe conditions
so that the aircraft will loiter at its current location for some period
of time (say one minute) and then proceed back towards home via a
pre-determined flight path. It may also include changes in airspeed,
changes in altitude, automatic landing or anything else that can be
programmed in a mission. If the failsafe event stops (for example GPS
lock is regained) then the aircraft will switch back to the mission item
it was previously flying towards and continue the mission.

This makes the AFS options only really appropriate for AUTO missions. If
you are primarily flying Plane in CRUISE mode or other modes then you
should use the standard failsafe options.

Enabling the AFS failsafe system
--------------------------------

To enable the AFS failsafe system you need to set the :ref:`AFS_ENABLE<AFS_ENABLE>` parameter to 1. The default is zero, which means all the other options are disabled.

Note that the AFS system is only built into Plane by default on higher
end autopilot boards like the PX4 and Pixhawk.

AFS Termination
---------------

The concept of "flight termination" is key to understanding the AFS
failsafe system. Termination is where the aircraft deliberately dives
into the ground by setting all control surfaces to maximum and throttle
is disarmed so as to enter a spin, or in the case of QuadPlane, there is also the option to immediately QLAND instead.

The AFS system will only start a termination if the :ref:`AFS_TERM_ACTION<AFS_TERM_ACTION>` is
set to the magic value 42 or 43. For any other value the AFS system will print
a message on the GCS console saying that it wants to terminate, but
won't actually change the control surfaces at all. Using a value other
than 42 or 43 is useful for test flights where you don't want the aircraft to
terminate on a failsafe event. A value of 42 immediately crashed the plane, while, if a QuadPlane, a value of 43 will immediately execute a QLAND.

Note that if :ref:`AFS_TERM_ACTION<AFS_TERM_ACTION>` is not set to 42 then other normal
failsafe code is still active, for example if you have a geofence
enabled then the aircraft will fly back to the geofence return point.

When enabled, the AFS termination system also sets up the secondary IO
micro-controller on the Pixhawk autopilot to terminate the aircraft if
communication is interrupted between the main FMU micro-controller and
the IO micro-controller, for example if the flight firmware crashes.

An AFS flight termination is not recoverable. Once your aircraft starts
a termination, there is no way to recover.

Types of Failsafe Events
------------------------

The AFS failsafe system supports five of types of failsafe events:

-  geofence breach
-  maximum pressure altitude breach
-  GPS loss
-  Ground station communications loss
-  barometer failure

Each of these types of failures has its own specific handling, which is
described below.

Geofence Breach
~~~~~~~~~~~~~~~

If a :ref:`geofence <geofencing>` is enabled then the AFS failsafe module
will monitor the aircraft for a breach of the boundaries of the geofence
(and lower and upper geofence altitudes if set). If a breach happens
then the AFS system will immediately terminate the flight (see
termination above).

In addition, if the total straight-line distance from arming point exceeds the :ref:`AFS_MAX_RANGE<AFS_MAX_RANGE>` distance, then termination will occur.

Maximum pressure altitude breach
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When sharing airspace with other aircraft it is usual practice to define
the available flight altitudes in terms of a common reference pressure,
typically QNH (a reference to "nautical height"). The QNH reference
pressure, measured in millibar, is distributed to all aircraft either
via a radio message or through aviation internet and weather sites.

Aircraft then use their barometer to measure the pressure relative to
that QNH pressure, which gives them an altitude reference which all
aircraft in the area should be using.

The AFS system is able to enforce a pressure altitude limit
by setting the QNH pressure in the :ref:`AFS_QNH_PRESSURE<AFS_QNH_PRESSURE>` parameter, as a
value in millibars. The pilot should then also set a pressure altitude
limit using the :ref:`AFS_AMSL_LIMIT<AFS_AMSL_LIMIT>` parameter (in meters). Note that this
pressure altitude limit is relative to sea level (AMSL stands for "above
mean sea level").

If both of these parameters are set then the AFS system fill monitor
pressure altitude and will initiate a termination if the pressure
altitude rises above the :ref:`AFS_AMSL_LIMIT<AFS_AMSL_LIMIT>`.

You need to be very careful to set the right :ref:`AFS_QNH_PRESSURE<AFS_QNH_PRESSURE>` for your
local conditions on the day of your flight, as the QNH pressure can be
very different on different days.

In addition to the QNH pressure limit, the AFS system also monitors the
health of your barometer. If the barometer is unhealthy for 5 seconds
then the AFS system will check the :ref:`AFS_AMSL_ERR_GPS<AFS_AMSL_ERR_GPS>` parameter. If it
is -1 (the default) then the aircraft will terminate immediately. If it
is not -1 then the AFS system will use the :ref:`AFS_AMSL_ERR_GPS<AFS_AMSL_ERR_GPS>` value as
a margin to add to the GPS height, and will allow the flight to continue
if the GPS altitude plus the :ref:`AFS_AMSL_ERR_GPS<AFS_AMSL_ERR_GPS>` value (in meters) is
below the :ref:`AFS_AMSL_LIMIT<AFS_AMSL_LIMIT>` value. The purpose of this margin is to
account for the inaccuracy of GPS altitudes. A value of 200 is
reasonable for safety to ensure the :ref:`AFS_AMSL_LIMIT<AFS_AMSL_LIMIT>` pressure altitude
is not breached.

GPS Loss
~~~~~~~~

The AFS system monitors the health of your GPS receivers throughout the
flight. If all of your available GPS receivers lose position lock then
this initiates a GPS failure failsafe.

When a GPS failure occurs (which is defined as loss of GPS lock for 3
seconds) the AFS system will look at the :ref:`AFS_WP_GPS_LOSS<AFS_WP_GPS_LOSS>` parameter.
This parameter species a waypoint number in your mission to use when a
GPS failure occurs. If :ref:`AFS_WP_GPS_LOSS<AFS_WP_GPS_LOSS>` is non-zero the aircraft will
change current waypoint to the waypoint number specified in :ref:`AFS_WP_GPS_LOSS<AFS_WP_GPS_LOSS>` . You should setup your mission so that the aircraft
will perform whatever actions you want on GPS loss. For example, you
could have a set of waypoints starting at number 10 which first loiter
on the spot for 30 seconds, and then proceed back to the airfield. You
would then set ::ref:`AFS_WP_GPS_LOSS<AFS_WP_GPS_LOSS>` to 10 to enable that part of the
mission on loss of GPS lock.

When setting up mission items for GPS lock it is sometimes useful to
include "loiter at the current location" waypoints. That is achieved by
setting both the latitude and longitude of LOITER mission commands to
zero.

If the GPS recovers after a GPS failsafe has started, then the aircraft
will automatically resume its mission where it left off.

If :ref:`AFS_MAX_GPS_LOSS<AFS_MAX_GPS_LOSS>` is set to a non-zero number, then it is used as a
maximum count of the number of GPS failures that will be allowed while
returning to the mission after GPS lock is re-established. This counter
is only incremented if the 2nd GPS failure happens at least 30 seconds
after the previous one (to account for a short period of GPS failure).

If during a period of GPS loss the aircraft also loses communications
with the ground station then this is termed a "dual loss", and the
aircraft will terminate if :ref:`AFS_DUAL_LOSS<AFS_DUAL_LOSS>` is 1.

Ground station communications loss
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The AFS system monitors the health of the link between your ground
station and your aircraft. It does this by looking for HEARTBEAT MAVLink
messages coming from the ground station.

If the aircraft does not receive a HEARTBEAT message for a period of 10
seconds then it enters a GCS failsafe state. It then looks for a
:ref:`AFS_WP_COMMS<AFS_WP_COMMS>` parameter, and if that is non-zero it will change the
current target waypoint to the one given in :ref:`AFS_WP_COMMS<AFS_WP_COMMS>`. You should
set up a section of your mission with whatever actions you want to take
on loss of communications.

If GPS lock is lost at the same time as GCS communications is lost then
that is considered a "dual loss", and the aircraft will immediately
terminate.

Note that the monitoring of HEARTBEAT messages only tells the autopilot
that it can see messages from the ground station. It does not mean the
ground station can see messages from the aircraft. So it is quite
possible for your ground station to be reporting loss of communication
while the aircraft is still receiving HEARTBEAT messages.

If :ref:`AFS_MAX_COM_LOSS<AFS_MAX_COM_LOSS>` is set to a non-zero number, then it is used as a
maximum count of the number of communication failures that will be
allowed while returning to the mission after communications is
re-established. This counter is only incremented if the 2nd communication
failure happens at least 30 seconds after the previous one (to account
for a short period of communications failure).

If during a period of GPS loss the aircraft also loses communications
with the ground station then this is termed a "dual loss", and the
aircraft will terminate if :ref:`AFS_DUAL_LOSS<AFS_DUAL_LOSS>` is 1.

RC Loss
~~~~~~~

If RC control is lost in for more than :ref:`AFS_RC_FAIL_TIME<AFS_RC_FAIL_TIME>` milliseconds, flight termination is activated. This termination mode is only enabled if :ref:`AFS_RC_FAIL_TIME<AFS_RC_FAIL_TIME>` is non-zero and :ref:`AFS_ENABLE<AFS_ENABLE>` is 1.
For the OBC rules it should be set to 1500 (giving 1.5 seconds).

If :ref:`AFS_RC_MAN_ONLY<AFS_RC_MAN_ONLY>` is set to 1 then this will only occur if in a manual mode. Otherwise, it will occur in any flight mode.

Monitoring the AFS system
~~~~~~~~~~~~~~~~~~~~~~~~~

The AFS system provides some additional parameters to make it easier to
monitor the health of the failsafe system using external electronics
(such as an external failsafe board).

The key parameters are:

-  :ref:`AFS_TERM_PIN<AFS_TERM_PIN>` : This is a digital pin which is set to a high
   voltage if termination is started. Note that this pin will go high on
   termination even if the :ref:`AFS_TERM_ACTION<AFS_TERM_ACTION>` parameter is not set to 42.
-  :ref:`AFS_HB_PIN<AFS_HB_PIN>` : This is a digital pin number for a pin which is
   toggled at a rate of 10Hz by the failsafe system. If termination
   occurs and a :ref:`AFS_TERM_PIN<AFS_TERM_PIN>` value is not set then the heartbeat pin
   will stop toggling.
-  :ref:`AFS_MAN_PIN<AFS_MAN_PIN>` : This is a digital pin number for a pin which goes
   high when the aircraft is in MANUAL mode. It may be useful with some
   external failsafe boards to detect manual mode and behave
   differently.

Manual Termination
------------------

Apart from automatic termination it is also important for the aircraft's
operator to be able to terminate the aircraft immediately if they think
the aircraft is a danger to people or other aircraft. To force an
immediate termination you should use the :ref:`AFS_TERMINATE<AFS_TERMINATE>` parameter. By
setting that parameter to 1 the aircraft will immediately terminate.

Example AFS failsafe mission
----------------------------

Setting up a AFS failsafe mission takes time, and needs to be done very
carefully. To help you understand what is possible you may find the
following example files useful

-  A `waypoint mission <https://github.com/tridge/cuav/blob/master/cuav/data/way.txt>`__
   for the 2014 Outback Challenge with waypoints for different AFS
   failures commented in the file
-  A `geofence file <https://github.com/tridge/cuav/blob/master/cuav/data/fence.txt>`__
   for the 2014 Outback Challenge

Testing the AFS system in SITL
------------------------------

It is highly recommended that you extensively test the AFS system using
the :ref:`SITL simulation system <dev:simulation-2>` before using it
on a real aircraft. You can simulate all types of in-flight failures
using the SIM\_ parameters. To start SITL in Kingaroy ready for OBC
testing you would use:

::

    sim_vehicle.py -L Kingaroy --console --map

The key parameters for failsafe testing in SITL are:

-  Test GPS failure: param set SIM_GPS_DISABLE 1
-  Test RC failure: param set SIM_RC_FAIL 1
-  Test comms failure: set heartbeat 0
-  Test fence failure: switch to CRUISE mode and fly across boundary
-  Test QNH failure: param set :ref:`AFS_AMSL_LIMIT<AFS_AMSL_LIMIT>` = 100

Additional tips for AFS failsafe users
--------------------------------------

You need to ensure that your geofence is enabled before takeoff. This
can either be done as part of your preflight checklist, or you could set
a :ref:`FENCE_CHANNEL<FENCE_CHANNEL>` and enable it from within your transmitter.  This ensures
that if your transmitter is out of range that the fence remains enabled.

Settings for Outback Challenge 2014
-----------------------------------

To be compliant with the OBC 2014 rules you should have the following
settings:

-  :ref:`AFS_ENABLE<AFS_ENABLE>` : 1
-  :ref:`AFS_WP_COMMS<AFS_WP_COMMS>` : waypoint number for OBC comms hold followed by two
   minute loiter, then return to airfield home
-  :ref:`AFS_WP_GPS_LOSS<AFS_WP_GPS_LOSS>` : waypoint number to loiter in place for 30
   seconds, followed by return to airfield home
-  :ref:`AFS_TERM_ACTION<AFS_TERM_ACTION>` : 42
-  :ref:`AFS_AMSL_LIMIT<AFS_AMSL_LIMIT>` : 914
-  :ref:`AFS_QNH_PRESSURE<AFS_QNH_PRESSURE>` : correct QNH pressure for the day
-  :ref:`AFS_RC_FAIL_TIME<AFS_RC_FAIL_TIME>` : 1500
-  :ref:`AFS_MAX_GPS_LOSS<AFS_MAX_GPS_LOSS>` : 2
-  :ref:`AFS_MAX_COM_LOSS<AFS_MAX_COM_LOSS>` : 2
