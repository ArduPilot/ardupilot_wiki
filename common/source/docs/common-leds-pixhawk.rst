.. _common-leds-pixhawk:

============
LEDs Meaning
============

This topic explains how to interpret the colours and flash sequence of
the main and/or external LED(s), if the :ref:`NTF_LED_OVERRIDE<NTF_LED_OVERRIDE>` parameter is set to its ``standard`` default setting. Some of the LED patterns have associated sound/tone
patterns as listed in :ref:`Sounds (Pixhawk) <common-sounds-pixhawkpx4>`.

Boards with 2 Notify LEDS
=========================

These boards have two leds on the board: LED A and LED B, normally, but not always, green and blue, respectively.

After boot, LED A usually shows system status, while LED B shows GPS status. Each sequence of flashes shown below repeats every second. So, LED A will flash with a 50% duty cycle at 4Hz , alternating with LED B, when the autopilot is in ESC calibration mode.

The states and events are shown in order of precedence. For example, LED A will flash the battery failsafe sequence even if RC is in failsafe also.

+-----------------------------------------------+--------------------+-----------------+
| STATE                                         | LED A              | LED B           |
+===============================================+====================+=================+
| Initializing (calibrating gyros,etc.)         |     |init|         |                 |
+-----------------------------------------------+--------------------+-----------------+
| Save Trim or ESC Calibration                  |     |savetrima|    | |savetrimb|     |
+-----------------------------------------------+--------------------+-----------------+
| Compass Calibration                           |     |compcala|     | |compcalb|      |
+-----------------------------------------------+--------------------+-----------------+
| | AUTOTUNE:                                   | |                  |                 |
| | Completed                                   | | |atc|            |                 |
| | Failed                                      | | |atf|            |                 |
+-----------------------------------------------+--------------------+-----------------+
| |  ARMED:                                     | |                  |                 |
| |  Battery failsafe                           | | |battfs|         |                 |
| |  RC or GCS failsafe                         | | |fs|             |                 |
| |  Readytofly                                 | | |armed|          |                 |
+-----------------------------------------------+--------------------+-----------------+
| |  DISARMED:                                  | |                  |                 |
| |  pre-arm checks failing                     | | |prearm|         |                 |
| |  ready to arm                               | | |ready|          |                 |
+-----------------------------------------------+--------------------+-----------------+
| |  GPS:                                       |                    | |               |
| |  No lock or No GPS                          |                    | | not lit       |
| |  Lock (flashs = # sats/2)                   |                    | | |gps| 8sats   |
+-----------------------------------------------+--------------------+-----------------+

.. |init| image:: ../../../images/initializinga.gif
  :width: 30pt

.. |savetrima| image:: ../../../images/savetrima.gif
  :width: 30pt

.. |savetrimb| image:: ../../../images/savetrimb.gif
  :width: 30pt

.. |compcala| image:: ../../../images/compasscala.gif
  :width: 30pt

.. |compcalb| image:: ../../../images/compasscalb.gif
  :width: 30pt
.. |atc| image::  ../../../images/autotunecomplete.gif
  :width: 30pt

.. |atf| image::  ../../../images/autotunefail.gif
  :width: 30pt

.. |armed| image::  ../../../images/armed.gif
  :width: 30pt

.. |fs| image::  ../../../images/controlfs.gif
  :width: 30pt

.. |battfs| image::  ../../../images/battfs.gif
  :width: 30pt

.. |prearm| image::  ../../../images/prearmfail.gif
  :width: 30pt

.. |ready| image::  ../../../images/readytoarm.gif
  :width: 30pt

.. |gps| image::  ../../../images/gps.gif
  :width: 30pt

RGB LEDS
========

The states and events are shown in order of precedence.

+-----------------------------------------------+--------------------+
| STATE                                         | LED                |
+===============================================+====================+
| Initializing (calibrating gyros,etc.)         |     |rgbinit|      |
+-----------------------------------------------+--------------------+
| Save Trim or ESC Calibration                  |     |rgbsavetrim|  |
+-----------------------------------------------+--------------------+
| |  Leak failsafe                              | | |rgbleakfs|      |
| |  EKF failsafe                               | | |rgbekffs|       |
| |  GPS glitch                                 | | |rgbgpsglitch|   |
| |  Radio/GCS/Batt failsafe                    | | |rgbcontrolfs|   |
+-----------------------------------------------+--------------------+
| |  ARMED:                                     | |                  |
| |  3Dfix                                      | | |rgbarmed|       |
| |  No valid GPS fix                           | | |rgbarmednogps|  |
+-----------------------------------------------+--------------------+
| |  DISARMED:                                  | |                  |
| |  pre-arm checks failing                     | | |rgbprearm|      |
| |  good DGPS fix                              | | |rgbready1|      |
| |  good GPS fix                               | | |rgbready2|      |
| |  bad GPS fix                                | | |rgbbadgps|      |
+-----------------------------------------------+--------------------+


.. |rgbinit| image:: ../../../images/rgb-initializing.gif
  :width: 30pt

.. |rgbsavetrim| image:: ../../../images/rgb-savetrim.gif
  :width: 30pt

.. |rgbleakfs| image:: ../../../images/rgb-leakfs.gif
  :width: 30pt

.. |rgbekffs| image:: ../../../images/rgb-ekffs.gif
  :width: 30pt

.. |rgbcontrolfs| image::  ../../../images/rgb-controlfs.gif
  :width: 30pt

.. |rgbarmed| image::  ../../../images/rgb-armed.gif
  :width: 30pt

.. |rgbarmednogps| image::  ../../../images/rgb-armed-nogps.gif
  :width: 30pt

.. |rgbgpsglitch| image::  ../../../images/rgb-gps-glitch.gif
  :width: 30pt

.. |rgbbadgps| image::  ../../../images/rgb-bad-gps.gif
  :width: 30pt

.. |rgbprearm| image::  ../../../images/rgb-prearmfail.gif
  :width: 30pt

.. |rgbready1| image::  ../../../images/rgb-good-dgps.gif
  :width: 30pt

.. |rgbready2| image::  ../../../images/rgb-good-gps.gif
  :width: 30pt

Video Overview
==============

..  youtube:: j-CMLrAwlco
    :width: 100%

LED/Buzzer Meanings
===================

**Flashing red and blue**: Initializing gyroscopes. Hold the vehicle
still and level while it initializes the sensors.

**Flashing blue**: Disarmed, no GPS lock found. Autopilot, loiter and
return-to-launch modes require GPS lock.

**Solid blue**: Armed with no GPS lock

**Flashing green**: Disarmed (ready to arm), GPS lock acquired. Quick
double tone when disarming from the armed state.

**Fast Flashing green**: Same as above but GPS is using SBAS (so should
have better position estimate).

**Solid green - with single long tone at time of arming:** Armed, GPS
lock acquired. Ready to fly!

**Double flashing yellow:** Failing pre-arm checks (system refuses to
arm).

**Single Flashing yellow:** Radio failsafe activated

**Flashing yellow - with quick beeping tone**: Battery failsafe
activated

**Flashing yellow and blue - with high-high-high-low tone sequence (dah-dah-dah-doh):** GPS glitch or GPS failsafe activated

**Flashing red and yellow - with rising tone:** EKF or Inertial Nav
failure

**Flashing Red, Blue and Green**: Copter ESC Calibration mode entered. See :ref:`esc-calibration` 

`SOS tone sequence <https://download.ardupilot.org/downloads/wiki/pixhawk_sound_files/NoSDCard_short.wav>`__ : SD Card missing (or other SD error like bad format etc.)

[copywiki destination="plane,copter,rover,blimp"]