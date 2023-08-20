.. _mower-hardware: 

========================
Hardware Recommendations
========================

This page lists proven hardware options and combinations for use on Rover mowers.

Add the interconnect cube pilot style component wiring diagram I created

Autopilots
==========

Orange Cube or Matek H743 is widley used on mowers.

Many of the mowers out there use the Orange Cubes and they have all the processing power and memory that the mower requires. The Matek H743 series have VERY similar processor and memory specs as the Cube Orange and Orange+. What you give up with the Matek boards is heated/vibration damped IMUs (not super important features for the mower application) and the convenient form factor of the Cubes. You actually gain a UART or two. A good compromise might be the ZealotH743, which is packaged into a very nice enclosure at a lower price point than the Cubes.
Some users have used other autopilots for the mower application as well, but the Orange Cube and the Matkek H743 allow advanced control abilities scripts as well as room to store 700 waypoints for complex mowing plans.  Other autopilots may not have the same capabilities.

GPS Hardware
============

2 SimpleRTK2B GPS boards or other  ZED-F9P GPS boards such as Sparkfun SMA- F9Ps are needed for the mower.  One board is the GPS for the mower determining the mower's position and the other board takes position readings with respect to known locations to calculate the direction the mower is pointed (aka Moving Base or GPS for Yaw).  People often ask if the moving base on the mower replaces the need for a base station.  The answer is "No".
The 3B Heading board will also work fine. It’s just hard to justify the cost over two Zed-F9P based modules.

RTK base station will be required unless you can connect to an NTRIP RTK correction service (most users end up building a base station),  If you build a base station you will need an additional ZED-F9P GPS board for the base station.

RTK GPS Configuration <mower-gps>

Radio Control
=============

Selecting a radio transmitter is difficult because a large number of them will work just fine and everyone has personal preferences and a budget they want to stay within.  It is recommended that you get a radio and receiver that has at least 10 channels.
Here are a few people are using: RadioMaster TX16S, FLYSKY FS-I6X with a 10 Channel receiver

Telemetry
=========

mRo SiK Telemetry Radio
Holybro Sik telemetry radio
RFD900 telemetry radio
Along with many others

The telemetry link from your Windows base computer to the mower is essential.  All communication from your base station computer to the mower Autopilot goes through this link.  It is not recommended to use WIFI instead of a radio for telemetry on a mower appliocation.


Servos
======

The drive servos are selected based on the amount of force that is needed to move the hydraulic control valve on the mower platform that is selected.  The servo mentioned here is for the very common HydroGear transaxle used on many mowers.  If you have a mower platform that is different you will have to determine what servo will work by researching what others have used or measuring the torque required.  There is a resistance spring on each of the hydraulic control valves that needs to be removed to reduce the torque required by the servo.  The resistance springs are only needed when driving with the manual drive handles.  The manual drive handles are disconnected when the servos are connected.  Manual drive handle operation is not compatible with the electronic drive system.  The new manual operation is now to flip a switch on the RC controller and drive with the joysticks.
 It is a multi-dimensional puzzle to figure out where to mount the drive servos. They need to be in a serviceable location, protected from the other moving parts and heat sources, be very firmly mounted, as well as being in the right place to get the movement needed.  This will likely require building custom mounting hardware and linkages. The AGFRC 100KG servo is a bigger more robust solution, having more torque and operating at a higher voltage.  The HiTec D845WP is a 45KG servo operating at a lower voltage (5-8v), also requiring a separate lower voltage power supply.  Both of these servos are Waterproof - Metal Gear servos for use in harsh environments.

Relays/Relay Boards
===================

Other hardware is needed to automate the engine throttle, mower deck blade control, and carburetor choke.  This part of the build gets very customized and every interface to these mower platforms is different.  Some common parts often used are as follows:

PWM relay bank, switches, fuses, and higher current relay for blade clutch control

Not sure what to do with this, but maybe some example pictures of things like PWM relay bank and higher amperage relays



Additional Hardware
===================
