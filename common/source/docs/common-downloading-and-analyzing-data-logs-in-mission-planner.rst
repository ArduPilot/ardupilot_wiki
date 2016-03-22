.. _common-downloading-and-analyzing-data-logs-in-mission-planner:

======================================================
Downloading and Analyzing Data Logs in Mission Planner
======================================================

:ref:`Dataflash logs <common-downloading-and-analyzing-data-logs-in-mission-planner>`
are stored on the Pixhawk (or APM2.x) onboard dataflash memory and can
be download after a flight. On Plane and Rover dataflash logs are
created soon after start-up. On Copter they are created after you first
arm the copter. This topic explains how to configure and access
Dataflash logs.

[copywiki destination="copter,plane,rover,planner"]

.. note::

   :ref:`Telemetry logs <common-mission-planner-telemetry-logs>` (also
   known as "tlogs") collect similar information to dataflash logs (see
   :ref:`Diagnosing problems using Logs <common-diagnosing-problems-using-logs>` for more information).

.. _common-downloading-and-analyzing-data-logs-in-mission-planner_downloading_logs_via_mavlink:

Downloading logs via MAVLink
============================

This method is available in AC3.2 (or higher).

-  Connect your vehicle to the mission planner using the micro USB cable
-  Open the Mission Planner's Flight Data screen
-  On the bottom left, select the "DataFlash Logs" tab and push the
   "Download DataFlash Log Via Mavlink" button

.. image:: ../../../images/Capture3.png
    :target: ../_images/Capture3.png

.. image:: ../../../images/Capture3.png
    :target: ../_images/Capture3.png

Then, select the log you want to download. This will save that log to
your MissionPlanner/logs directory, in a folder named after the vehicle
type, such as QUADCOPTER.

Automatic Analysis of logs
==========================

.. figure:: ../../../images/MissionPlanner_AutomaticLogAnalysis_Buttons.png
   :target: ../_images/MissionPlanner_AutomaticLogAnalysis_Buttons.png

   Mission Planner: Start LogAnalysis

The simplest analysis is to generate a basic automated report that will
highlight common problem areas. For that, click on "Log Analysis"
and select a log that you've already saved to the MissionPlanner/logs
directory.  They will be in folders named after the vehicle type, such
as QUADCOPTER or ROVER. Once you pick the log you want, it will generate
a report that looks like this:

.. image:: ../../../images/Capture3.png
    :target: ../_images/Capture3.png

Manually review a log
=====================

For more detailed analysis, click on "Review a Log" and select a log
that you've already saved to the MissionPlanner/logs directory.  Once
again, they will be in folders named after the vehicle type, such as
QUADCOPTER or ROVER. Once you pick the log you want, you will get charts
such as the below. The basic format of the dataflash is:

-  Line numbers appear on the very left side of the viewer
-  Software version and board type appear at the top
-  FMT messages are next which tell the mission planner the column
   headers for each message type
-  PARM rows which show each parameter (in the order in which they
   appear in the eeprom) along with their value at the beginning of the
   flight
-  Flight data messages including GPS, IMU, etc.

.. image:: ../../../images/mp_dataflash_format.png
    :target: ../_images/mp_dataflash_format.png

Graph any flight data by first clicking on the appropriate row, you
should see the column headers update appropriately. Next find the column
you wish to graph, click on it and then push the "Graph this data"
button. In the example above the ATT's Roll-In and Roll data have been
graphed. The mouse's scroll wheel can be used to zoom in or out. You may
also select an area of the graph to zoom in on it. Zoom out by
right-mouse-button clicking and selecting "Set Scale to Default".
`Here's <http://www.diydrones.com/profiles/blog/show?id=705844%3ABlogPost%3A801607>`__
a mini tutorial on using this feature. You may also filter on just the
first column (the flight data message type) by clicking on the first
column and selecting the message type from the drop-down. This is very
useful especially for viewing the different flight modes (called "MODE"
messages) used during the mission. Click the first column again but
press "Cancel" to clear the filter.

.. image:: ../../../images/MissionPlanner_CLI_openDataflashFilter.png
    :target: ../_images/MissionPlanner_CLI_openDataflashFilter.png

Setting what data you want recorded
===================================

The LOG_BITMASK parameter controls what message types are recorded in
the dataflash.  Recent versions of the mission planner and ardupilot
allow individual messages to be enabled/disabled from the MP's
Config/Tuning, Standard Params screen.

.. image:: ../../../images/mp_dataflash_log_bitmask.png
    :target: ../_images/mp_dataflash_log_bitmask.png

Earlier versions of MP/ardupilot allow selecting the message using a
drop-down as shown below. 

.. image:: ../../../images/mp_log_bitmask.png
    :target: ../_images/mp_log_bitmask.png

.. _common-downloading-and-analyzing-data-logs-in-mission-planner_message_details_copter_specific:

Message Details (Copter specific)
=================================

**ATT (attitude information):**

+-----------+--------------------------------------------------------------------------------------------------------+
| RollIn    | The pilot's desired roll angle in centi-degrees (roll left is negative, right is positive)             |
+-----------+--------------------------------------------------------------------------------------------------------+
| Roll      | The vehicle's actual roll in centi-degrees (roll left is negative, right is positive)                  |
+-----------+--------------------------------------------------------------------------------------------------------+
| PitchIn   | The pilot's desired pitch angle in centi-degrees (pitch forward is negative, pitch back is positive)   |
+-----------+--------------------------------------------------------------------------------------------------------+
| Pitch     | The vehicle's actual pitch angle in centi-degrees (roll left is negative, right is positive)           |
+-----------+--------------------------------------------------------------------------------------------------------+
| YawIn     | Tthe pilot's desired yaw rate as a number from -4500 ~ 4500 (not in deg/sec, clockwise is positive)    |
+-----------+--------------------------------------------------------------------------------------------------------+
| Yaw       | The vehicles actual heading in centi-degrees with 0 = north                                            |
+-----------+--------------------------------------------------------------------------------------------------------+
| NavYaw    | The desired heading in centi-degrees                                                                   |
+-----------+--------------------------------------------------------------------------------------------------------+

**ATUN (auto tune overview):**

+--------------------------------------+--------------------------------------+
|                                      | Axis: 0 = Roll, 1 = Pitch            |
+--------------------------------------+--------------------------------------+
| TuneStep                             | 0 = Returning towards Level (before  |
|                                      | or after a test), 1 = Testing (i.e.  |
|                                      | performing a twitch to test          |
|                                      | response), 2 = Updating gains        |
|                                      | (twitch completed and gains          |
|                                      | adjusted)                            |
+--------------------------------------+--------------------------------------+
| RateMin                              | Minimum recorded rate during this    |
|                                      | test                                 |
+--------------------------------------+--------------------------------------+
| RateMax                              | Maximum recorded rate during this    |
|                                      | test                                 |
+--------------------------------------+--------------------------------------+
| RPGain                               | Rate P gain value being tested       |
+--------------------------------------+--------------------------------------+
| RDGain                               | Rate D gain value being tested       |
+--------------------------------------+--------------------------------------+
| SPGain                               | Stabilize P gain being tested        |
+--------------------------------------+--------------------------------------+

**ATDE (auto tune step details):**

+---------+-------------------------------------------------------------------+
| Angle   | Angle of the copter in centi-degrees for the axis being testedx   |
+---------+-------------------------------------------------------------------+
| Rate    | Rate of rotation of the copter for the axis being tested          |
+---------+-------------------------------------------------------------------+

**CAM (time and position when camera shutter was activated):**

+-----------+-----------------------------------------------------------------------+
| GPSTime   | The GPS reported time since epoch in milliseconds                     |
+-----------+-----------------------------------------------------------------------+
| Lat       | The accelerometer + GPS latitude estimate                             |
+-----------+-----------------------------------------------------------------------+
| Lng       | The accelerometer + GPS longitude estimate                            |
+-----------+-----------------------------------------------------------------------+
| Alt       | The accelerometer + barometer estimated altitude in cm above ground   |
+-----------+-----------------------------------------------------------------------+
| Roll      | The vehicle roll angle in centi-degrees                               |
+-----------+-----------------------------------------------------------------------+
| Pitch     | The vehicle pitch angle in centi-degrees                              |
+-----------+-----------------------------------------------------------------------+
| Yaw       | The vehicle's heading in centi-degrees                                |
+-----------+-----------------------------------------------------------------------+

**CMD (commands received from the ground station or executed as part of
a mission):**

+--------+----------------------------------------------------------------------------------------+
| CTot   | The total number of commands in the mission                                            |
+--------+----------------------------------------------------------------------------------------+
| CNum   | This command's number in the mission (0 is always home, 1 is the first command, etc)   |
+--------+----------------------------------------------------------------------------------------+
| CId    | The `mavlink message id <https://pixhawk.ethz.ch/mavlink/>`__                          |
+--------+----------------------------------------------------------------------------------------+
| Copt   | The option parameter (used for many different purposes)                                |
+--------+----------------------------------------------------------------------------------------+
| Prm1   | The command's parameter (used for many different purposes)                             |
+--------+----------------------------------------------------------------------------------------+
| Alt    | The command's altitude in meters                                                       |
+--------+----------------------------------------------------------------------------------------+
| Lat    | The command's latitude position                                                        |
+--------+----------------------------------------------------------------------------------------+
| Lng    | The command's longitude position                                                       |
+--------+----------------------------------------------------------------------------------------+

**COMPASS (raw compass, offset and compassmot compensation values):**

.. raw:: html

   <table>
   <tbody>
   <tr>
   <th>Field</th>
   <th>Description</th>
   </tr>
   <tr>
   <td>MagX, MagY. MagZ</td>
   <td>Raw magnetic field values for x, y and z axis</td>
   </tr>
   <tr>
   <td>OfsX, OfsY, OfsZ</td>
   <td>Raw magnetic offsets (will only change if COMPASS_LEARN parameter is 1)</td>
   </tr>
   <tr>
   <td>MOfsX, MOfsY, MOfsZ</td>
   <td>Compassmot compensation for throttle or current</td>
   </tr>
   </tbody>
   </table>

**CURRENT (battery voltage, current and board voltage information):**

.. raw:: html

   <table>
   <tbody>
   <tr>
   <th>FIELD</th>
   <th>DESCRIPTION</th>
   </tr>
   <tr>
   <td>Thr</td>
   <td>Pilot input throttle from 0 ~ 1000</td>
   </tr>
   <tr>
   <td>ThrInt</td>
   <td>Integrated throttle (i.e. sum of total throttle output for this flight)</td>
   </tr>
   <tr>
   <td>Volt</td>
   <td>Battery voltage in volts \* 100</td>
   </tr>
   <tr>
   <td>Curr</td>
   <td>Current drawn from the battery in amps \* 100</td>
   </tr>
   <tr>
   <td>Vcc</td>
   <td>Board voltage</td>
   </tr>
   <tr>
   <td>CurrTot</td>
   <td>Total current drawn from battery</td>
   </tr>
   </tbody>
   </table>

**CTUN (throttle and altitude information):**

.. raw:: html

   <table>
   <tbody>
   <tr>
   <th>FIELD</th>
   <th>DESCRIPTION</th>
   </tr>
   <tr>
   <td>ThrIn</td>
   <td>The pilot's throttle in as a number from 0 to 1000</td>
   </tr>
   <tr>
   <td>SonAlt</td>
   <td>The altitude above ground according to the sonar</td>
   </tr>
   <tr>
   <td>BarAlt</td>
   <td>The altitude above ground according to the barometer</td>
   </tr>
   <tr>
   <td>WPAlt</td>
   <td>The desired altitude while in AltHold, Loiter, RTL or Auto flight modes</td>
   </tr>
   <tr>
   <td>NavThr</td>
   <td>Not used</td>
   </tr>
   <tr>
   <td>AngBst</td>
   <td>Throttle increase (from 0 ~ 1000) as a result of the copter leaning over
   (automatically added to all pilot and autopilot throttle to reduce
   altitude loss while leaning)   </td>
   </tr>
   <tr>
   <td>CRate</td>
   <td>Accelerometer + baro climb rate estimate in cm/s</td>
   </tr>
   <tr>
   <td>ThrOut</td>
   <td>Final throttle output sent to the motors (from 0 ~ 1000). Normally equal
   to ThrIn+AngBst while in stabilize mode.   </td>
   </tr>
   <tr>
   <td>DCRate</td>
   <td>Pilot desired climb rate in cm/s</td>
   </tr>
   </tbody>
   </table>

**D32, DU32 (single data values which are either signed 32bit integers
or unsigned 32bit integers):**

.. raw:: html

   <table>
   <tbody>
   <tr>
   <th>FIELD</th>
   <th>DESCRIPTION</th>
   </tr>
   <tr>
   <td>id</td>
   <td>Identification number for the variable.  There are only two possible values:
   <ul>
   <li>7 = bit mask of internal state.  The meaning of individual bits can
   be found in `Copter.cpps’s definition of the ap structure <https://github.com/diydrones/ardupilot/blob/master/ArduCopter/ArduCopter.cpp#L361>`__./
   <li>9 = simple mode's initial heading in centi-degrees</li>
   </ul>
   </td>
   </tr>
   </tbody>
   </table>

**EKF (Extended Kalman Filter**):

:ref:`Log information here <dev:extended-kalman-filter_interpreting_log_data>`
(Dev Wiki). Overview :ref:`here <common-apm-navigation-extended-kalman-filter-overview>`.

**ERR (an error message):**

SubSystem and Error codes listed below

.. raw:: html

   <table>
   <tbody>
   <tr>
   <th>Error</th>
   <th>DESCRIPTION</th>
   </tr>
   <tr>
   <td>1: Main</td>
   <td> (never used)</td>
   </tr>
   <tr>
   <td>2: Radio</td>
   <td>
   <ul>
   <li>ECode 1: “Late Frame” which means the APM’s onboard ppm encoder did not provide an update for at least 2 seconds
   </li>ECode 0: error resolved which means the ppm encoder started providing data again</li>
   </ul>
   </td>
   </tr>
   <tr>
   <td>3: Compass</td>
   <td>
   <ul>
   <li>ECode 1: the compass failed to initialise (likely a hardware issue)</li>
   <li>ECode 2: failure while trying to read a single value from the compass (probably a hardware issue)</li>
   <li>ECode 0: above errors resolved</li>
   </ul>
   </td>
   </tr>
   <tr>
   <td>4: Optical flow</td>
   <td>Ecode 1: failed to initialise (likely a hardware issue)</td>
   </tr>
   <tr>
   <td>5: Throttle failsafe</td>
   <td>
   <ul>
   <li>ECode 1: throttle dropped below FS_THR_VALUE meaning likely loss of contact between RX/TX</li>
   <li>ECode 0: above error resolve meaning RX/TX contact likely restored</li>
   </ul>
   </td>
   </tr>
   <tr>
   <td>6: Battery failsafe</td>
   <td>ECode 1: battery voltage dropped below LOW_VOLT or total battery capacity used exceeded BATT_CAPACITY</td>
   </tr>
   <tr>
   <td>7: GPS failsafe</td>
   <td>
   <ul>
   <li>ECode 1: GPS lock lost for at least 5 seconds</li>
   <li>ECode 0: GPS lock restored</li>
   </ul>
   </td>
   </tr>
   <tr>
   <td>8: GCS (Ground station) failsafe</td>
   <td>
   <ul>
   <li>ECode 1: updates from ground station joystick lost for at least 5 seconds</li>
   <li>ECode 0: updates from ground station restored</li>
   </ul>
   </td>
   </tr>
   <tr>
   <td>9: Fence</td>
   <td>
   <ul>
   <li>ECode 1: altitude fence breached</li>
   <li>ECode 2: circular fence breached</li>
   <li>ECode 3: both altitude and circular fences breached</li>
   <li>ECode 0: vehicle is back within the fences</li>
   </ul>
   </td>
   </tr>
   <tr>
   <td>10: Flight Mode</td>
   <td>
   ECode 0 ~ 17: the vehicle was unable to enter the desired flight mode</li>
   (0=Stabilize, 1=Acro, 2=AltHold, 3=Auto, 4=Guided, 5=Loiter, 6=RTL,
   7=Circle, 8=Position, 9=Land, 10=OF_Loiter, 11=Drift, 13=Sport,
   14=Flip, 15=AutoTune, 16=PosHold, 17=Brake)
   </td>
   </tr>
   <tr>
   <td>11: GPS</td>
   <td>
   <ul>
   <li>ECode 2: GPS Glitch</li>
   <li>ECode 0: GPS Glitch cleared</li>
   </ul>
   </td>
   </tr>
   <tr>
   <td>12: Crash Check</td>
   <td>ECode 1: Crash detected</td>
   </tr>
   </tbody>
   </table>

**EV: (an event number)**. The full list of possible events can be found
in `defines.h <https://github.com/diydrones/ardupilot/blob/master/ArduCopter/defines.h#L291>`__
but the most common are:

.. raw:: html

   <table>
   <tbody>
   <tr>
   <th>Event Number</th>
   <th>DESCRIPTION</th>
   </tr>
   <tr>
   <td>10</td>
   <td>Armed</td>
   </tr>
   <tr>
   <td>11</td>
   <td>Disarmed</td>
   </tr>
   <tr>
   <td>15</td>
   <td>Auto Armed (pilot has raised throttle above zero and autopilot is free to take control of throttle)</td>
   </tr>
   <tr>
   <td>16</td>
   <td>TakeOff</td>
   </tr>
   <tr>
   <td>18</td>
   <td>Land Complete</td>
   </tr>
   <tr>
   <td>25</td>
   <td>Set Home (home location coordinates have been capture)</td>
   </tr>
   </tbody>
   </table>

**GPS:**

.. raw:: html

   <table>
   <tbody>
   <tr>
   <th>FIELD</th>
   <th>DESCRIPTION</th>
   </tr>
   <tr>
   <td>Status</td>
   <td>0 = no GPS, 1 = GPS but no fix, 2 = GPS with 2D fix, 3 = GPS with 3D fix   </td>
   </tr>
   <tr>
   <td>Time</td>
   <td>The GPS reported time since epoch in milliseconds</td>
   </tr>
   <tr>
   <td>NSats</td>
   <td>The number of satellites current being used</td>
   </tr>
   <tr>
   <td>`HDop <http://en.wikipedia.org/wiki/Dilution_of_precision_%28GPS%29>`__:</td>
   <td>A measure of gps precision (1.5 is good, >2.0 is not so good)</td>
   </tr>
   <tr>
   <td>Lat</td>
   <td>Lattitude according to the GPS</td>
   </tr>
   <tr>
   <td>Lng</td>
   <td>Longitude according to the GPS</td>
   </tr>
   <tr>
   <td>RelAlt</td>
   <td>Accelerometer + Baro altitude in meters</td>
   </tr>
   <tr>
   <td>Alt</td>
   <td>GPS reported altitude (not used by the flight controller)</td>
   </tr>
   <tr>
   <td>SPD</td>
   <td>Horizontal ground speed in m/s</td>
   </tr>
   <tr>
   <td>GCrs</td>
   <td>Ground course in degrees (0 = north)</td>
   </tr>
   </tbody>
   </table>

**IMU (accelerometer and gyro information):**

.. raw:: html

   <table>
   <tbody>
   <tr>
   <th>FIELD</th>
   <th>DESCRIPTION</th>
   </tr>
   <tr>
   <td>GyrX, GyrY, GyrZ</td>
   <td>The raw gyro rotation rates in degrees/second</td>
   </tr>
   <tr>
   <td>AccX, AccY, AccZ</td>
   <td>The raw accelerometer values in m/s/s</td>
   </tr>
   </tbody>
   </table>

**Mode (flight mode):**

.. raw:: html

   <table>
   <tbody>
   <tr>
   <th>FIELD</th>
   <th>DESCRIPTION</th>
   </tr>
   <tr>
   <td>Mode</td>
   <td>The flight mode displayed as a string (i.e. STABILIZE, LOITER, etc)</td>
   </tr>
   <tr>
   <td>ThrCrs</td>
   <td>Throttle cruise (from 0 ~ 1000) which is the autopilot's best guess as to what throttle is required to maintain a stable hover</td>
   </tr>
   </tbody>
   </table>

**NTUN (navigation information):**

.. raw:: html

   <table>
   <tbody>
   <tr>
   <th>FIELD</th>
   <th>DESCRIPTION</th>
   </tr>
   <tr>
   <td>WPDst</td>
   <td>Distance to the next waypoint (or loiter target) in cm. Only updated
   while in Loiter, RTL, Auto flight modes.   </td>
   </tr>
   <tr>
   <td>WPBrg</td>
   <td>Bearing to the next waypoint in degrees</td>
   </tr>
   <tr>
   <td>PErX</td>
   <td>Distance to intermediate target between copter and the next waypoint in
   the latitude direction   </td>
   </tr>
   <tr>
   <td>PErY</td>
   <td>Distance to intermediate target between copter and the next waypoint in the longitude direction</td>
   </tr>
   <tr>
   <td>DVelX</td>
   <td>Desired velocity in cm/s in the latitude direction</td>
   </tr>
   <tr>
   <td>DVelY</td>
   <td>Desired velocity in cm/s in the longitude direction</td>
   </tr>
   <tr>
   <td>VelX</td>
   <td>Actual accelerometer + gps velocity estimate in the latitude direction</td>
   </tr>
   <tr>
   <td>VelY</td>
   <td>Actual accelerometer + gps velocity estimate in the longitude direction</td>
   </tr>
   <tr>
   <td>DAcX</td>
   <td>Desired acceleration in cm/s/s in the latitude direction</td>
   </tr>
   <tr>
   <td>DAcY</td>
   <td>Desired acceleration in cm/s/s in the longitude direction</td>
   </tr>
   <tr>
   <td>DRol</td>
   <td>Desired roll angle in centi-degrees</td>
   </tr>
   <tr>
   <td>DPit</td>
   <td>Desired pitch angle in centi-degrees</td>
   </tr>
   </tbody>
   </table>

**PM (performance monitoring):**

.. raw:: html

   <table>
   <tbody>
   <tr>
   <th>Field</th>
   <th>DESCRIPTION</th>
   </tr>
   <tr>
   <td>RenCnt</td>
   <td>DCM renormalization count - a high number may indicate problems in DCM (extremely rare)</td>
   </tr>
   <tr>
   <td>RenBlw</td>
   <td>DCM renormalization blow-up count - how many times DCM had to completely
   rebuild the DCM matrix since the last PM. Normally innocuous but a
   number that constantly grows may be an indication that DCM is having
   troubles calculating the attitude (extremely rare)   </td>
   </tr>
   <tr>
   <td>FixCnt</td>
   <td>The number of GPS fixes received since the last PM message was received (it's hard to imagine how this would be useful)</td>
   </tr>
   <tr>
   <td>NLon</td>
   <td>Number of long running main loops (i.e. loops that take more than 5% longer than the 10ms they should)</td>
   </tr>
   <tr>
   <td>NLoop</td>
   <td>The total number of loops since the last PM message was displayed. This
   allows you to calculate the percentage of slow running loops (which
   should never be higher than 15%). Note that the value will depend on the
   autopilot clock speed.</td>
   </tr>
   <tr>
   <td>MaxT</td>
   <td>The maximum time that any loop took since the last PM message. This
   should be close to 10,000 but will be up to 6,000,000 during the
   interval where the motors are armed   </td>
   </tr>
   <tr>
   <td>PMT</td>
   <td>A number that increments each time a heart beat is received from the ground station</td>
   </tr>
   <tr>
   <td>I2CErr</td>
   <td>The number of I2C errors since the last PM message. Any I2C errors may
   indicate a problem on the I2C bus which may in turn slow down the main
   loop and cause performance problems.   </td>
   </tr>
   </tbody>
   </table>

**RCOUT (pwm output to individual RC outputs):**

RC1, RC2, etc : pwm command sent from flight controller to the
esc/motor/RC output

Viewing KMZ FILES
=================

When you download the dataflash log files from the APM/PX4 it will
automatically create a KMZ file (file with extension .kmz). This file
can be opened with Google Earth (just double click the file) to view
your flight in Google Earth. Please see the instructions on the
:ref:`Telemetry Logs Page <common-mission-planner-telemetry-logs_creating_3d_images_of_the_flight_path>`
for additional details.

Video tutorials
===============

..  youtube:: 62TmGiwFiDU
    :width: 100%

..  youtube:: IcVlJCR8N2g
    :width: 100%

