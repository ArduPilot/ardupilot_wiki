.. _airframe-disco:

============================
Setup Guide for Parrot Disco
============================

The Parrot Disco is a lightweight delta-wing with built-in camera and
good flight characteristics. This page will help you get setup with
ArduPilot running on your Disco.

Building Firmware
=================

To build the ArduPilot firmware you should use the waf build system,
which is included as part of ArduPilot. The command to build APM:Plane
for Disco is:

  ./waf configure --board disco
  ./waf plane

this will give you a file build/disco/bin/arduplane that needs to be
installed on your Disco.

Installation
============

ArduPilot is usually installed in /data/ftp/internal_000/APM. You
should use adb to create that directory and put a copy of arduplane
from the build step in the directory.

You will also need to modify the /etc/init.d/rcS_mode_default startup
script to start ArduPilot. An example of a modified startup script to
start ArduPilot if it is installed is available here:

  http://uav.tridgell.net/Disco/installation/rcS_mode_default

That will start ArduPilot is there is a start_ardupilot.sh script in
the APM directory.

You will also need to install the start_ardupilot.sh script. A copy of
a suitable script is here:

 http://uav.tridgell.net/Disco/installation/start_ardupilot.sh

that starts the Disco FAN and then loops running ArduPilot. The loop
is so you can restart ArduPilot with a MAVLink preflight-reboot
request without rebooting the Disco.

The startup script assumes that your Disco appears on the
192.168.42.0/24 network for WiFi and 192.168.43.0/24 network for USB
networking. It tells ArduPilot to announce itself as a MAVLink device
on those two networks. It will pick up the first MAVLink capable
ground station that connects on UDP port 14550 on those networks.

Ground Station
==============

For configuring your Disco you will need a ground station software
package (GCS). There are several choices for use with Disco:

* MissionPlanner (windows only)
* APM Planner (cross platform)
* QGroundControl (cross platform)
* MAVProxy (cross platform, experts only)

They are all capable of controlling all aspects of the Disco setup and
flight, including autonomous missions.
  
Transmitter Setup
=================

The usual way to fly a Disco with ArduPilot is with a SBUS receiver
and matching transmitter. You can also fly without a transmitter, but
that is only recommanded for advanced users who are very familiar with
automated flight control with ArduPilot. We hope to support the Disco
WiFi based transmitter in ArduPilot in the future.

For controlling Disco using a SBUS receiver and R/C transmitter you
need to setup how the elevon mixing is done. The combination of
channel directions on your transmitter must match the parameters setup
in the Disco.

The most common issue with R/C setup on the Disco is what reversal you
have on your R/C transmitter for elevator (pitch) input. Some
transmitters default to pulling back on the pitch stick giving a
higher PWM value whereas other transmitter manufacturers use a
convention where pulling back on the pitch stick gives a lower PWM
value.

With the 3.7.0 and earlier release of APM:Plane you need to change
your transmitter to match the expected direction for the Disco
parameters. That expected direction is that pulling back on the pitch
stick produces a lower PWM output from the transmitter. For that setup
you need:

* ELEVON_OUTPUT 2
* RC2_REV -1

For the 3.7.1 release and later there are additional options for
ELEVON_OUTPUT which allow for your transmitter to have either
reversal. If you have 3.7.1 or later installed and your transmitter
produces a higher value on the pitch channel when pulling back on the
stick then you need:

* ELEVON_OUTPUT 6
* RC2_REV 1

Loading Parameters
==================

You will need to load a set of parameters suitable for the Disco using
your GCS. A recommended set of parameters to start with is here:

  http://uav.tridgell.net/Disco/installation/disco-defaults.parm

 
Compass Calibration
===================

You need to calibrate the compass in your Disco before you fly. This
must be done with the hatch in place due to the magnetic catch on the
hatch.

Each GCS choice has an option to start a compass calibration. Please
choose on-board compass calibration for your GCS and follow the
prompts.

Note that you may find you need to raise COMPASS_CAL_FIT to allow
successful calibration of the Disco, as the magnetic setup of the
Disco hardware is not ideal and won't produce a perfect fit. We
recommend setting COMPASS_CAL_FIT to 20.

Accelerometer Calibration
=========================

You also need to perform an accelerometer calibration. Please follow
the prompts in your GCS for the accelerometer calibration
procedure. This will only need to be performed once.

Airspeed Calibration
====================

Before each flight you should perform an airspeed offset calibration
as the airspeed sensor will vary in its zero value between power
cycles.

You should loosely cover the pitot tube that is built into the power
switch and choose the pre-flight airspeed calibration option in your
GCS.

Stabilisation Check
===================

Before each flight you should check the stabilization of the Disco by
changing to FBWA mode and checking the following:

* roll the Disco to the right. The right elevon should go down, the
  left elevon should go up
* roll the Disco to the left. The left elevon should go down, the
  right elevon should go up
* pitch the nose up. Both elevons should go down
* pitch the nose down. Both elevons should go up

Next you should check for correct transmitter control with the Disco
held level.

* input right roll on the transmitter. The left elevon should go down
  and the right elevon should go up
* input left roll on the transmitter. The right elevon should go down
  and the left elevon should go up
* pull back on the pitch (elevator) stick on the transmitter. Both
  elevons should go up.
* push forward on the pitch (elevator) stick on the transmitter. Both
  elevons should go down.

Takeoff
=======

The Disco has a very low stall speed which makes it easy to launch in
a wide variety of ways. Some recommended ways are:

* a side launch where you hold a wing close to the fuselage, and launch
  the aircraft forward. An example is shown here:

..  youtube:: 493782HmSqc
    :width: 100%

* a forward throw launch, as shown here:

..  youtube:: nDMZibc_CNo
    :width: 100%

Always launch into the wind, and be careful to keep your hand clear of
the propeller.

Also note that you can configure Disco for "shake to start", to start
the motor when the airframe senses a shaking motion. That is set by
the TKOFF_THR_MINACC=4 parameter in the parameter file linked above.

You can see a "shake to start" example here:

..  youtube:: d2kEPkCueYY
    :width: 100%
