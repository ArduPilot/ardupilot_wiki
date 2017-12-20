.. _rover-failsafes:

=========
Failsafes
=========

Rover supports three failsafe mechanisms as described below.

Radio Failsafe (aka Throttle Failsafe)
--------------------------------------

.. image:: ../images/rover-failsafe-rc.jpg
    :target: ../_images/rover-failsafe-rc.jpg

This failsafe is triggered if the connection between the user's transmitter and the receiver on the vehicle is lost for at least :ref:`FS_TIMEOUT <FS_TIMEOUT>` seconds.

- the loss of transmitter/receiver connection is detected by:

  - no signals being sent from the receiver to the autopilot board OR
  - the throttle channel (normally input channel 3) value falling below the :ref:`FS_THR_VALUE <FS_THR_VALUE>` parmeter value

- set :ref:`FS_THR_ENABLE <FS_THR_ENABLE>` to "1" to enable this failsafe
- if :ref:`FS_ACTION <FS_ACTION>` is "1", the vehicle will :ref:`RTL <rtl-mode>` to home, if "2" the vehicle will :ref:`Hold <hold-mode>`
- once the transmitter/receiver connection is restored, the user may use the transmitter's mode switch to re-take control of the vehicle in :ref:`Manual <manual-mode>` (or any other mode)
    
GCS Failsafe (aka Telemetry Failsafe)
-------------------------------------

This failsafe is triggered if the vehicle stops receiving `heartbeat messages <http://mavlink.org/messages/common#HEARTBEAT>`__ from the ground station for at least :ref:`FS_TIMEOUT <FS_TIMEOUT>` seconds.

- set :ref:`FS_GCS_ENABLE <FS_GCS_ENABLE>` to "1" to enable this failsafe
- if :ref:`FS_ACTION <FS_ACTION>` is "1", the vehicle will :ref:`RTL <rtl-mode>` to home, if "2" the vehicle will :ref:`Hold <hold-mode>`
- use the transmitter's mode switch to re-take control of the vehicle in :ref:`Manual <manual-mode>` (or any other mode)

Crash Check
-----------

If enabled by setting the :ref:`FS_CRASH_CHECK <FS_CRASH_CHECK>` parameter to "1" (for :ref:`Hold <hold-mode>`) or "2" (for :ref:`Hold <hold-mode>` and Disarm) this failsafe will switch the vehicle to Hold and then (optionally) disarm the vehicle if all the following are true for at least 2 seconds:

- the vehicle is in :ref:`Auto <auto-mode>`, :ref:`Guided <guided-mode>`, :ref:`RTL <rtl-mode>` or :ref:`SmartRTL <smartrtl-mode>` mode
- velocity falls below 0.08m/s (i.e. 8cm/s)
- the vehicle is turning at less than 4.5 deg/s
- demanded throttle to the motors (from the pilot or autopilot) is at least 5%
