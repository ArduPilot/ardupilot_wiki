.. _balance_bot-configure:

=======================
Configuration and Setup
=======================

1) Connections, Firmware and Calibration
========================================

#. Refer to :ref:`autopilot system assembly instructions <rover-autopilot-assembly-instructions>` for making connections between the autopilot board and each of these components:

    - Power Module
    - ESC/Motor Drive
    - Motors
    - Wheel Encoders
    - RC Receiver
    - GPS(optional)
    - Telemetry(optional)

#. :ref:`Install GCS<common-install-gcs>` (Mission Planner recommended) and :ref:`upload rover firmware<common-loading-firmware-onto-pixhawk>`, if ArduPilot firmware already is installed, or :ref:`Loading Firmware onto boards without existing ArduPilot firmware (first time only) <common-loading-firmware-onto-chibios-only-boards>`
#. Perform all the :ref:`hardware calibration<rover-code-configuration>` steps for:

    - :ref:`Accelerometer<common-accelerometer-calibration>`
    - :ref:`Compass<common-compass-calibration-in-mission-planner>`
    - :ref:`Radio<common-radio-control-calibration>` 
    - :ref:`RC Mode Setup<common-rc-transmitter-flight-mode-configuration>` (Add Manual and Acro Modes)

2) Motor, ESC, Wheel Encoder Configuration
==========================================

#. Follow the :ref:`instructions<rover-motor-and-servo-configuration-skid>` to setup skid steering drive
#. Configure motor drive/ESC type for :ref:`brushed motors<common-brushed-motors>` or :ref:`brushless motors<rover-motor-and-servo-configuration>`
#. Verify connections and settings using the :ref:`motor test<rover-motor-and-servo-configuration-testing>` tool
#. Configure :ref:`wheel encoders<wheel-encoder>` 

3) Additional Parameter Configuration
=====================================
The following parameters must be set to these specified values:

- :ref:`FRAME_CLASS<FRAME_CLASS>` = 3 (For firmware to recognize vehicle as Balance Bot)
- :ref:`MOT_SLEWRATE<MOT_SLEWRATE>` = 0 (Do not account for motor slew)
- :ref:`FS_CRASH_CHECK<FS_CRASH_CHECK>` = 1 (Enable Crash Check)

.. _balance_bot-configure-throttle:

4) Minimum Throttle
===================
Many motors and ESCs have a dead zone. This is the zone between the zero throttle value and the throttle value at which the motor starts to move. This can be compensated by setting minimum throttle in the firwamre.

.. tip:: Remove wheels before proceeding

To fix the dead zone, open the motor test window in Mission Planner, as mentioned :ref:`here<rover-motor-and-servo-configuration-testing>`.  Find the minimum throttle value at which the motor turns on and set the parameter :ref:`MOT_THR_MIN<MOT_THR_MIN>` to that value. Now the motor should start at 1% throttle.

5) Arming
=========
The vehicle must be armed for the wheels to start moving. Check the :ref:`rover arming page<arming-your-rover>` for more details. 

.. warning:: This is simply an arming test. The vehicle will have to be tuned before it is ready to run.

.. tip:: Remove wheels before proceeding. 

#. Set a :ref:`transmitter switch<common-auxiliary-functions>` for arming. Ensure the channel used for the switch has been :ref:`calibrated<common-radio-control-calibration>`. To configure a channel for arming, for example channel 7, then set the parameter:

    - :ref:`RC7_OPTION<RC7_OPTION>` =41 (Sets function of channel 7 as arming/disarming)

#. Connect the battery. Connect the autopilot board to GCS via USB or telemetry.

#. Keep the vehicle upright and then arm it. If arming is not successful check the error message on the GCS and identify the problem from the :ref:`rover arming page<arming-your-rover>` .

#. After the vehicle arms, pitch it forward and back manually(Use hands, not the RC transmitter). The motors must turn in the direction of pitch. 

#. Proceed to the :ref:`Control Modes<balance_bot-modes>` and :ref:`tuning<balance_bot-tuning>` section if the above steps were successful.



















