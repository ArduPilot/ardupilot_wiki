.. _common-am32-escs:

===================
AM32 Drone CAN ESCs
===================

.. image:: ../../../images/am32-dronecan-escs-title.jpg
    :width: 450px

The `AM32 ESC firmware <https://github.com/am32-firmware/AM32>`__ is open source ESC firmware for use on multicopters and rovers.  DroneCAN support was added in late 2024

Where to Buy
------------

AM32 ESCs that support DroneCAN

- `VimDrones ESC Nano <https://dev.vimdrones.com/products/vimdrones_esc_nano/>`__
- `VimDrones ESC Development Board <https://dev.vimdrones.com/products/vimdrones_esc_dev/>`__

More ESCs may be found from the `AM32 targets.h <https://github.com/am32-firmware/AM32/blob/main/Inc/targets.h>`__ file (seach for _CAN)

ArduPilot Configuration
=======================

To enable communication with the AM32 ESCs using DroneCAN, set the following parameters

- Set :ref:`CAN_P1_DRIVER <CAN_P1_DRIVER>` = 1 (First driver) to specify that the ESCs are connected to the CAN1 port
- Set :ref:`CAN_D1_PROTOCOL <CAN_D1_PROTOCOL>` = 1 (DroneCAN)
[site wiki="copter,rover"]
- Set :ref:`MOT_PWM_MIN <MOT_PWM_MIN>` = 1000 and :ref:`MOT_PWM_MAX <MOT_PWM_MAX>` = 2000 so ArduPilot uses an output range that matches the ESCs input range
[/site]
[site wiki="plane"]
- Set ``SERVOx_MIN`` = 1000 and ``SERVOx_MAX`` = 2000 for each ESC connected (``x`` corresponds to the ESC number) so ArduPilot uses an output range that matches the ESCs input range
[/site]
- Set ``SERVOx_FUNCTION`` to each motor channel (e.g. 33 - 40 for motors 1 - 8). This is automatically configured when setting frame class/type. If a motor channel isn't assigned to a servo output, commands won't be sent to the associated ESC.

By default, all configured motor channels are used to send control commands to the ESCs. These control messages are transmitted by the autopilot at a default rate of 50Hz. These parameters can be modified using:

- :ref:`CAN_D1_PC_ESC_BM <CAN_D1_PC_ESC_BM>` is a bitmask that determines which ESC (motor) channels are transmitted over CAN
- :ref:`CAN_D1_PC_ESC_RT <CAN_D1_PC_ESC_RT>` determines the rate (Hz) at which commands are sent out to the ESCs

ESC Configuration
=================

The AM32 ESC can be setup using the `AM32 configurator <https://am32.ca/configurator>`__

Logging and Reporting
---------------------

ESCs RPM, voltage, current and temperature are recorded in the autopilot's onboard log and reported in real-time to the ground station

.. image:: ../../../images/dshot-realtime-esc-telem-in-mp.jpg
    :target: ../_images/dshot-realtime-esc-telem-in-mp.jpg
    :width: 450px
