.. _rover-tuning-process:

===========================
Tuning Process Instructions
===========================

This page describes the overall process to tune a Rover.  Before attempting these instructions please complete the :ref:`First Time Setup <apmrover-setup>` including:

- :ref:`RC calibration <common-radio-control-calibration>`
- :ref:`Accelerometer <common-accelerometer-calibration>` and :ref:`Compass Calibration <common-compass-calibration-in-mission-planner>`
- :ref:`Motor and Servo calibration <rover-motor-and-servo-configuration>`
- :ref:`Test the Motor Direction <rover-motor-and-servo-configuration-testing>` and :ref:`Minimum Throttle <rover-motor-and-servo-min-throttle>`

Setup the :ref:`Transmitter flight mode switch <common-rc-transmitter-flight-mode-configuration>` so the vehicle can be driven in :ref:`Manual <manual-mode>` and :ref:`Acro <acro-mode>` modes.

Check that the vehicle can be driven in Manual mode.

Tune the Speed and Throttle Controller
--------------------------------------

Follow the instructions on the :ref:`Tuning Speed and Throttle <rover-tuning-throttle-and-speed>` page.  The high level steps are:

1. Drive the vehicle in Manual mode in a straight line at between 50% and 80% throttle and use the "Learn Cruise Speed" :ref:`auxiliary switch <common-auxiliary-functions>` to capture the :ref:`CRUISE_THROTTLE <CRUISE_THROTTLE>` and :ref:`CRUISE_SPEED <CRUISE_SPEED>` parameter values

2. Test and adjust the speed controller gains in real-time while driving in Acro mode

    - Set :ref:`GCS_PID_MASK <GCS_PID_MASK>` to 2 (Throttle) to send PID information to the ground station
    - On the ground station graph the "piddesired" and "pidachieved" values
    - Drive the vehicle at various speeds in Acro mode and compare how well pidachieved follows piddesired
    - Adjust :ref:`ATC_SPEED_P <ATC_SPEED_P>`, :ref:`ATC_SPEED_I <ATC_SPEED_I>` up or down so that piddesired follows pidachieved

3. If the vehicle accelerates too quickly or too slowly adjust the :ref:`ATC_ACCEL_MAX <ATC_ACCEL_MAX>`

Tune the Turn Rate Controller
-----------------------------

Follow the instructions on the :ref:`Tuning Steering Rate <rover-tuning-steering-rate>` page.  The high level steps are:

1. Set the :ref:`ACRO_TURN_RATE <ACRO_TURN_RATE>` parameter to be roughly what the vehicle's maximum turn rate is in deg/sec
2. Test and adjust the turn rate controller gains in real-time while driving in Acro mode

    - Set :ref:`GCS_PID_MASK <GCS_PID_MASK>` to 1 (Steering) to send PID information to the ground station
    - On the ground station graph the "piddesired" and "pidachieved" values
    - Drive the vehicle in Acro mode at a medium speed making various wide and tight turns and compare how well pidachieved follows piddesired.  The most important parameter to get right is :ref:`ATC_STR_RAT_FF <ATC_STR_RAT_FF>`

3. Set :ref:`ATC_STR_RAT_MAX <ATC_STR_RAT_MAX>` to the :ref:`ACRO_TURN_RATE <ACRO_TURN_RATE>` value and optionally reduce :ref:`ACRO_TURN_RATE <ACRO_TURN_RATE>`

Configure Pivot Turns (Skid Steering vehicles only)
---------------------------------------------------

If the vehicle uses :ref:`Skid Steering <rover-motor-and-servo-connections-skid-steering>` follow the instructions on the :ref:`Tuning Pivot Turns <rover-tuning-pivot-turns>` page

Tune the Navigation Controller
------------------------------

Follow the instructions on the :ref:`Tuning Navigation <rover-tuning-navigation>` page
