.. _common-downloads_advanced_user_tools:

==============================
DOWNLOADS: Advanced User Tools
==============================

The advanced users tools `can be downloaded from here <http://download.ardupilot.org/downloads/wiki/advanced_user_tools/>`__.

.. note::

    Not all of these tools are "up to date". For example, vehicle binaries and 
    GCS software should be downloaded from their respective locations in the 
    `firmware server <http://firmware.ardupilot.org/>`__.

The rest of the article provides an overview of some of the files and the tool licensing.


Tools
=====

Tarot Gimbal firmware
---------------------

Tarot Gimbal firmware and configuration UI for Windows.

* `ZYX-BMGC-EN_V1.5.zip <http://download.ardupilot.org/downloads/wiki/advanced_user_tools/ZYX-BMGC-EN_V1.5.zip>`__ (v1.5)
* `ZYX-BMGC-EN_V1.4.zip <http://download.ardupilot.org/downloads/wiki/advanced_user_tools/ZYX-BMGC-EN_V1.4.zip>`__ (v1.4)


Copter Firmware
---------------

.. warning::

    Not necessarily the latest!
    
* `ArduCopter_APM_1.0_Firmware_3.2.1.zip <http://download.ardupilot.org/downloads/wiki/advanced_user_tools/ArduCopter_APM_1.0_Firmware_3.2.1.zip>`__
* `ArduCopter_APM_2.0_Firmware_3.2.1.zip <http://download.ardupilot.org/downloads/wiki/advanced_user_tools/ArduCopter_APM_2.0_Firmware_3.2.1.zip>`__


Rover Firmware
--------------

.. warning::

    Not necessarily the latest!
    
* `APMrover2-2.45.zip <http://download.ardupilot.org/downloads/wiki/advanced_user_tools/APMrover2-2.45.zip>`__
* `APMrover2-2.46.zip <http://download.ardupilot.org/downloads/wiki/advanced_user_tools/APMrover2-2.46.zip>`__
* `APMrover2-2.47.zip <http://download.ardupilot.org/downloads/wiki/advanced_user_tools/APMrover2-2.47.zip>`__



3DR PPM Encoder V3 Manual
-------------------------

`PPM-Encoder-V3-Manual.pdf <http://download.ardupilot.org/downloads/wiki/advanced_user_tools/PPM-Encoder-V3-Manual.pdf>`__

VERSION	1
DATE POSTED	January 29, 2014

User manual for the third version of the 3DR PPM Encoder module


USB Driver for Pixhawk Boards
-----------------------------

`px4_win_drivers.zip <http://firmware.us.ardupilot.org/Tools/PX4_Windows_Driver/px4_win_drivers.zip>`__

Drivers for Pixhawk Boards
Date Posted: May 15, 2013


3DR-uBlox configuration file
----------------------------

`3DR-Ublox.txt <http://download.ardupilot.org/downloads/wiki/advanced_user_tools/3DR-Ublox.txt>`__

DATE POSTED	November 24, 2013

This file is an extract of :
ArduPlane-2.73/libraries/AP_GPS/config
You can use it for configuring CRIUS CN-06 V2 for example.


APM2 / Ublox passthrough
------------------------

`GPS_UBLOX_passthrough_APM2.hex <http://download.ardupilot.org/downloads/wiki/advanced_user_tools/GPS_UBLOX_passthrough_APM2.hex>`__

VERSION	1.0
DATE POSTED	November 24, 2013

This executable allows the APM2.x to be used as a pass through to the GPS so that U-Center can be used to examine and change the UBlox’s configuration


ArduCAM OSD Config (For MinimOSD's Firmware v2.0)
-------------------------------------------------

`Config_OSD_20.zip <http://download.ardupilot.org/downloads/wiki/advanced_user_tools/Config_OSD_20.zip>`__

DATE POSTED	April 29, 2013


Important NOTE: After install the “ArduCAM OSD Config Tool” remember that you need to update the Character Set. A new one is available on the install folder. From now on it will be always there as “OSD_Charset.mcm”.

ArduCopter APM 1.0 Firmware
---------------------------

VERSION	3.2.1
DATE POSTED	March 18, 2015

ArduCopter v3.2.1 firmware for APM 1.0. This is the last build of ArduCopter that still fits on the APM 1.0 boards.


ArduCopter APM 2.x Firmware
---------------------------

VERSION	3.2.1
DATE POSTED	March 18, 2015


ArduCopter v3.2.1 firmware for APM 2.x. This is the last build of ArduCopter that still fits on the APM 2.x boards.


ArduPPM Encoder 3DR Manual v2.3.16.pdf
--------------------------------------

`PPM-Encoder-3DR-Manual-v2.3.16.pdf <http://download.ardupilot.org/downloads/wiki/advanced_user_tools/PPM-Encoder-3DR-Manual-v2.3.16.pdf>`__

VERSION	2.3.16
DATE POSTED	April 26, 2013

Manual for the standalone PPM Encoder by 3DR.
This manual covers firmware version 2.3.16.


ArduPPM v2.3.16 ATMega32U2 firmware for APM 2.x (recommended APM2.x)
--------------------------------------------------------------------

* `ArduPPM_v2.3.16_ATMega32U2.hex_.zip <http://download.ardupilot.org/downloads/wiki/advanced_user_tools/ArduPPM_v2.3.16_ATMega32U2.hex_.zip>`__

.. tip:: Recommended for all APM 2.x users

DATE POSTED	April 29, 2013



– New interrupt system that handles certain Futaba receivers better
(simultaneous changes on groups of R/C channels in fast intervals)
– Adapted behaviour in case of channel loss:

  - ch1, roll: set to center (1500us)
  - ch2, pitch: set to center (1500us)
  - ch3, throttle: set to low (900us)
  - ch4, yaw: set to center (1500us)
  - ch5-ch8: retain last value


APM 2.x LED STATUS:
+++++++++++++++++++

* - RX – OFF = No input signal detected
* - RX – SLOW TOGGLE = Input signal OK
* - RX – FAST TOGGLE = Invalid input signal(s) detected
* - RX – ON = Input signal(s) lost during flight and fail-safe activated
* - TX – OFF= PPM output disabled
* - TX – FAST TOGGLE = PPM output enabled
* - TX – SLOW TOGGLE = PPM pass-trough mode

SERVO INPUT (PWM) MODE:
+++++++++++++++++++++++

* – PPM output will not be enabled unless a input signal has been detected and verified
* – Verified inputs are lost during operation (lose servo wire or receiver malfunction):
* + The last known value of the lost input channel is kept for ~1 second
* + If the lost input channel is not restored within ~1 second, it will be set to the default fail-safe value (for channel 1-4) or kept at the last value (for channel 5-8)
* – Lost channel signal is restored:
* + Normal channel operation is restored using the valid input signal

PPM PASS-THROUGH MODE (signal pin 2&3 shorted):
+++++++++++++++++++++++++++++++++++++++++++++++

* – PPM output will not be enabled unless a input signal has been detected
* – Active signal on input channel 1 has been detected:
* + Any input level changes will be passed directly to the PPM output (PPM pass-trough)
* + If no input level changes are detected withing 250ms:
* + PPM output is enabled and default fail-safe values for all eight channels transmitted
* + Input level change detected again, PPM fail-safe output is terminated and normal PPM pass-through operation is restored




ArduPPM v2.3.16 ATMega328p firmware for APM1.x Copter and standalone PPM encoder. 
---------------------------------------------------------------------------------

`ArduPPM_v2.3.16_ATMega328p_for_ArduCopter.hex_.zip <http://download.ardupilot.org/downloads/wiki/advanced_user_tools/ArduPPM_v2.3.16_ATMega328p_for_ArduCopter.hex_.zip>`__

.. warning::

    **Do not use with APM 2.x**
    
    DATE POSTED	April 29, 2013


This version is dedicated to ArduCopter (CH8 radio passthrough mode disabled).

Changes:
++++++++

– New interrupt system that handles certain Futaba receivers better
(simultaneous changes on groups of R/C channels in fast intervals)
– Adapted behaviour in case of channel loss:

  - ch1, roll: set to center (1500μs)
  - ch2, pitch: set to center (1500μs)
  - ch3, throttle: set to low (900μs)
  - ch4, yaw: set to center (1500μs)

  - ch5-ch8: retain last value


LED STATUS:
+++++++++++

Normal mode:

– Error condition (All channels lost or throttle channel lost): blue LED blinks very fast
– Normal behaviour: blue LED blinks according to throttle position





ArduPPM v2.3.16 ATMega328p firmware for APM1.x Plane and standalone PPM encoder.
--------------------------------------------------------------------------------

`ArduPPM_v2.3.16_ATMega328p_for_ArduPlane.hex_.zip <http://download.ardupilot.org/downloads/wiki/advanced_user_tools/ArduPPM_v2.3.16_ATMega328p_for_ArduPlane.hex_.zip>`__

DATE POSTED	April 29, 2013

.. warning::

    Do not use with APM 2.x
    
This version is dedicated to ArduPlane (or ArduRover / Arduboat) (CH8 radio passthrough mode).
For Arducopter it is better to use the dedicated version so that CH8 can be fully used without risk to trig radio passthrough mode.

Changes:
++++++++

– New interrupt system that handles certain Futaba receivers better
(simultaneous changes on groups of R/C channels in fast intervals)

– Adapted behaviour in case of channel loss:

ch1, roll: set to center (1500μs)
ch2, pitch: set to center (1500μs)
ch3, throttle: set to low (900μs)
ch4, yaw: set to center (1500μs)

ch5-ch8: retain last value


LED STATUS:
+++++++++++

Normal mode:

– Error condition (All channels lost or throttle channel lost): blue LED blinks very fast
– Normal behaviour: blue LED blinks according to throttle position

Radio Passthrough mode (for ArduPlane only):

– If throttle position < 1200 μs, status LED is off 
- If throttle position > 1200 μs, status LED is on



ArduPPM v2.3.16 ATMega32U2 firmware for APM 2.x 
-----------------------------------------------

DATE POSTED	April 29, 2013

ArduPPM v2.3.16 ATMega32U2 firmware for APM 2.x

– New interrupt system that handles certain Futaba receivers better
(simultaneous changes on groups of R/C channels in fast intervals)
– Adapted behaviour in case of channel loss:

  - ch1, roll: set to center (1500us)
  - ch2, pitch: set to center (1500us)
  - ch3, throttle: set to low (900us)
  - ch4, yaw: set to center (1500us)

ch5-ch8: retain last value


APM 2.x LED STATUS:
+++++++++++++++++++

* RX – OFF = No input signal detected
* RX – SLOW TOGGLE = Input signal OK
* RX – FAST TOGGLE = Invalid input signal(s) detected
* RX – ON = Input signal(s) lost during flight and fail-safe activated
* TX – OFF= PPM output disabled
* TX – FAST TOGGLE = PPM output enabled
* TX – SLOW TOGGLE = PPM pass-trough mode

SERVO INPUT (PWM) MODE:
+++++++++++++++++++++++

– PPM output will not be enabled unless a input signal has been detected and verified
– Verified inputs are lost during operation (lose servo wire or receiver malfunction):
+ The last known value of the lost input channel is kept for ~1 second
+ If the lost input channel is not restored within ~1 second, it will be set to the default fail-safe value (for channel 1-4) or kept at the last value (for channel 5-8)
– Lost channel signal is restored:
+ Normal channel operation is restored using the valid input signal

PPM PASS-THROUGH MODE (signal pin 2&3 shorted):
+++++++++++++++++++++++++++++++++++++++++++++++

* – PPM output will not be enabled unless a input signal has been detected
* – Active signal on input channel 1 has been detected:
* + Any input level changes will be passed directly to the PPM output (PPM pass-trough)
* + If no input level changes are detected withing 250ms:
* + PPM output is enabled and default fail-safe values for all eight channels transmitted
* + Input level change detected again, PPM fail-safe output is terminated and normal PPM pass-through operation is restored

MediaTek GPS firmware update and utility
----------------------------------------

`MTK_DIYdrones.zip <http://download.ardupilot.org/downloads/wiki/advanced_user_tools/MTK_DIYdrones.zip>`__

DATE POSTED	April 29, 2013


MinimOSD Firmware v2.0
----------------------

`MinimOSD_20.hex_.zip <http://download.ardupilot.org/downloads/wiki/advanced_user_tools/MinimOSD_20.hex_.zip>`__

DATE POSTED	April 29, 2013

Firmware 2.0 for MinimOSD. More powerful than ever!

Important NOTE: You need to install the newest “ArduCAM OSD Config Tool” and also update the Character Set. A new one is available on the Config Tool’s install folder. From now on it will be always there as “OSD_Charset.mcm”.


MinimOSD Firmware v2.2 (Beta) - New Flight Modes
------------------------------------------------

`MinimOSD_22_hex.zip <http://download.ardupilot.org/downloads/wiki/advanced_user_tools/MinimOSD_22_hex.zip>`__


DATE POSTED	May 8, 2014

Firmware 2.2 for MinimOSD. New Flight-modes added for APM Plane and APM Copter.

Important NOTE: If you were using a firmware older than 2.0, remember of installing the newest “ArduCAM OSD Config Tool” and also update the Character Set. A new one is available on the Config Tool’s install folder. From now on it will be always there as “OSD_Charset.mcm”.


OpticalFlow test sketch / APM2
------------------------------

`AP_OpticalFlow_test.hex <http://download.ardupilot.org/downloads/wiki/advanced_user_tools/AP_OpticalFlow_test.hex>`__

VERSION	1.0
DATE POSTED	December 5, 2013

Optical Flow test sketch for APM2


PX4Flow-KLT
-----------

`px4flow-klt-06Dec2014.zip <http://download.ardupilot.org/downloads/wiki/advanced_user_tools/px4flow-klt-06Dec2014.zip>`__

VERSION	Beta-06Dec2014
DATE POSTED	December 6, 2014

PX4Flow sensor firmware using Lucas-Kanade method for use with ArduPilot


USB Driver for APM 2 
--------------------

`APM_Arduino_Drivers.zip <http://download.ardupilot.org/downloads/wiki/advanced_user_tools/APM_Arduino_Drivers.zip>`__

DATE POSTED	April 25, 2013

Arduino USB driver for the Atmega32u2 USB interface chip on APM 2


License
=======

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License 
as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
See the `GNU General Public License <http://www.gnu.org/licenses/gpl.html>`__ for more details.

Safety
======

Operating a powered vehicle of any kind can be a lot of fun. 
However, nothing will ruin your day at the park more quickly than an accident or running afoul of the law. 
Since we want you to have a great experience, please make sure that you do all of the following:

* Operate within all local laws and regulations. 
  For example, in the United States, current regulations require you to operate most UAVs under 400 
  foot above ground level, within line of site, and away from obstructions and populated areas. 
  Since these regulations vary from place to place, even within the same country, ensure that 
  you understand what you need to do to stay compliant.
* Never operate the vehicle or software in a way that could be dangerous to you, other people, or property. 
  Propellers, while rotating, could easily cut you; if a UAV fell on a person or object, 
  it could cause injury; a UAV caught in power lines could cause an outage. 
  As Ben Franklin said, “An ounce of prevention is worth a pound of cure.”
* Always keep in mind that software and hardware failures happen. 
  Although we design our products to minimize such issues, you should always operate with the understanding that 
  a failure could occur at any point of time and without warning. 
  As such, you should take the appropriate precautions to minimize danger in case of failure.
* Never use the software or hardware for manned vehicles. 
  The software and hardware we provide is only for use in unmanned vehicles.


[copywiki destination="copter,plane,rover,planner,planner2,antennatracker,dev,ardupilot"]
