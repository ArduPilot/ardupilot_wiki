.. _common-hitec-uavcan-servos:

=================================
Hitec UAVCAN Servo Configuration
=================================

This article details the UAVCAN Servo set-up from Hitec.  It includes three sections:

-  Servo Firmware Update Procedure
-  Servo Configuration using the Hitec UAVCAN Servo Configuration Utility
-  ArduPilot Configuration for UAVCAN Servos



Overview
========

UAVCAN is a lightweight protocol designed for reliable communication
in aerospace and robotic applications via CAN bus.
The UAVCAN network is a decentralized peer network, where each peer
(node) has a unique numeric identifier - node ID and that is only one
parameter needs to be set for basic setup.

Detailed description of protocol can be found at https://uavcan.org/

Hitec Commercial Solutions has developed a line of its actuators that can be used in either `UAVCAN v0 (aka v0.9) <https://github.com/UAVCAN/public_regulated_data_types/tree/legacy-v0/uavcan/equipment/actuator>`__, CAN2.0a, or CAN2.0b modes of operation.
For the purposes of this documentation, a `MD950TW-UAVCAN <https://hitecnology.com/actuators/md-series/md950mw-20mm-coreless-titanium-gear/product>`__ servo will be shown in pictures.

Current Hitec UAVCAN Servos Available
=====================================

-  `MD950TW <https://hitecnology.com/actuators/md-series/md950mw-20mm-coreless-titanium-gear/product>`_
-  `MD65MG <https://hitecnology.com/actuators/md-series/md65mg-12mm-stantard-composite-gear/product>`_
-  `MD70MH <https://hitecnology.com/actuators/md-series/md70mh-12mm-standard-metal-gear/product>`_
-  `MD85MG <https://hitecnology.com/actuators/md-series/hsi-m5085mg-industrial-grade-standard-servo/product>`_
-  `MD245MW <https://hitecnology.com/actuators/md-series/md245mw-17mm-metal-gear-wmagnetic-encoder-servo-actuator/product>`_
-  `MD250MW <https://hitecnology.com/actuators/md-series/md250mw-15mm-metal-gear-wmagnetic-encoder-servo-actuator/product>`_
-  `MD89MW <https://hitecnology.com/actuators/md-series/d-89mw-32bit-ultra-torque-micro-servo/product>`_
-  `SG33BLS/BLT <https://hitecnology.com/actuators/sg-series/sg33blt-xxxx-xxxx-servo/product>`_
-  SG20GL (Coming Soon , probably Aug 2020)


Hitec DPC-CAN Adapter Node
==========================

This device is required for PC-to-Servo communications for the purposes of firmware update and also configuration:
    .. image:: ../../../images/hitec-uavcan-servos-dpccan1.JPEG
        :target: ../_images/hitec-uavcan-servos-dpccan1.JPEG
        :width: 450px
        :alt: DPC-CAN Dongle
    .. image:: ../../../images/hitec-uavcan-servos-dpccan2.JPEG
        :target: ../_images/hitec-uavcan-servos-dpccan2.JPEG
        :width: 450px
        :alt: DPC-CAN Dongle and Servo

Hitec UAVCAN Servo Breakout Board
=================================

.. warning::
    Powering servos via the AutoPilot's CAN ports is not supported.  The power demands the servo can place on its power source greatly exceed the power supplied to the autopilot.  This is no different than isolating power from the autopilot's servo rail, and care must be taken to follow this recommendation in to UAVCAN Servos.  

In an effort to make integration easier, Hitec has developed these breakout boards that have solder tabs for a power source, such as a BEC or 2S LiPo, as well as two JST-GH connectors for connecting the breakout boards to the autopilot as well as sharing the bus with other peripherals.  
   .. image:: ../../../images/hitec-uavcan-servos-breakout1.JPEG
       :width: 450px
       :alt: UAVCAN Servo Breakout Board Prototype


   .. image:: ../../../images/hitec-uavcan-servos-breakoutandautopilot.JPEG
       :width: 450px

Servo Firmware Update Procedure
===============================
1.	Connect the servo to the DPC-CAN device as well as servo power.  Connect the DPC-CAN device via USB to your Windows computer.  Note the COM port associated with the DPC-CAN.

   .. image:: ../../../images/hitec-uavcan-servos-DPC-CAN-servo-power.JPEG
       :width: 450px

2.	Open the DPC-CAN Update tool, select the appropriate COM port, and press [Open].  You should see the text box populate with the DPC-CAN-V34 version as noted below:

   .. image:: ../../../images/hitec-uavcan-servos-fwupdate1.jpg
       :width: 450px

3.	Press [Load Patch] and browse to the update file and then click [Open] once you have the file selected.:

   .. image:: ../../../images/hitec-uavcan-servos-fwupdate2.jpg
       :width: 450px

4.	With the patch selected, you can now press [Erase App]

   .. image:: ../../../images/hitec-uavcan-servos-fwupdate3.jpg
       :width: 450px

5.	With the original firmware erased, you now click the [Download] button to begin the downloading of the firmware from your computer to the Servo

   .. image:: ../../../images/hitec-uavcan-servos-fwupdate4.jpg
       :width: 450px

6.	With a successful firmware install, the only thing remaining now is to press the [Go App] button.

   .. image:: ../../../images/hitec-uavcan-servos-fwupdate5.jpg
       :width: 450px

7.	With the [Go App] button pressed, data will begin streaming.  The firmware is now operational on the servo.  You can move on to Servo Configuration below.

   .. image:: ../../../images/hitec-uavcan-servos-fwupdate6.jpg
       :width: 450px

Servo Configuration
===================
1.	Connect the servo to the DPC-CAN device as well as servo power.  Connect the DPC-CAN device via USB to your Windows computer.  Note the COM port associated with the DPC-CAN.

   .. image:: ../../../images/hitec-uavcan-servos-config1.png
       :width: 450px

2.	With the correct COM port selected, press [Open] and verify the DPC-CAN-VER text field populates with the correct data.  Assuming it does, press the [Auto Scan] button to find the servo 

   .. image:: ../../../images/hitec-uavcan-servos-config2.png
       :width: 450px

3.	In the “SERVO Configuration” area in the top right section of the app, press [All] to select all the check boxes on that tab, and press the [READ] button to read those values from the servo in to the app
4.	The next step is to enter some configurations for this servo.  For the purposes of this exercise, this documentation assumes this servo will be “Servo 2” in ArduPilot.  It will also assume ArduPilot is on the bus as Node ID 10, which is ArduPilot's default UAVCAN configuration.  These instructions also assume you want a data stream rate of 50Hz but you can change this to suit your requirements.

-    Check the checkbox for CAN/Node ID, enter "10" in the text field, and press [SET] next to it
-    Check the checkbox for SERVO ID, enter "2" in the text field (if you are configuring for a different servo number, this is where you set that value), and press [SET] next to it
-    check the checkbox for Stream Mode, select the pull down to enable streaming, and press the [SET] button next to it
-    check the checkbox for Stream Time[ms] and enter 20 (you can set this to whatever value works for your system, 20ms is 50hz logging rate), and press [SET] next to it
-    press [Save]

   .. image:: ../../../images/hitec-uavcan-servos-config2b.png
       :width: 450px

5.  The next thing you'll want to change is how far the servo travels for a given input command.  By default, some of these Hitec UAVCAN servos will travel as far as -150 degrees all the way to +150 degrees for a total travel of 300 degrees.  For most ArduPilot-related applications, a 90-degree full-travel with 45 degrees on each side of zero will be the standard requirement, but you should configure this based on the needs of your system.  

-   First step is to update the following two fields to -45.00 and 45.00 

   .. image:: ../../../images/hitec-uavcan-servos-config3.png
       :width: 450px

-   Press that section’s [Left] button to swing the servo to its “Min” value and note the number it displays, in this case 6144
-   Next press the [Right] button and note the value it displays, in this case 10240 
-   In the “Servo Configuration” section, select the “Mode” tab and enter those values for the “POSITION MAX_LIMIT” and “POSITION MIN_LIMIT” fields, press each of their corresponding [SET] buttons, press [SAVE] and finally reboot the servo by pressing the [SERVO RESET] (Reminder, it probably just says [SERVO] on your screen like above)

   .. image:: ../../../images/hitec-uavcan-servos-config4.png
       :width: 450px

-   When the servo reboots, use the left/right buttons under the word "Test" to command unitless values of -1.000 and +1.000 to validate that those values are giving -45 and +45 degrees of servo travel.  Using these left/right buttons simulate ArduPilot sending full-swing commands in unitless values of -1 to +1.  Note: You may be required to update the "CAN ID" and "SERVO ID" under the DPC-CAN Configuration section of the app so that the app is now talking to the servo on its new Servo ID, as well as from CAN ID Node 10.

   .. image:: ../../../images/hitec-uavcan-servos-config6.png
       :width: 450px

You can now move to ArduPilot Configuration.

ArduPilot Configuration for UAVCAN Servos
=========================================
This procedure will cover AutoPilot configuration of the Hitec UAVCAN Servo.  This document is a continuation of the previous document and comes with the following assumptions:
-   ArduPilot will be NODE 10 on the UAVCAN Bus
-   Servo will be Servo 2, which is going to be configured as the Elevator servo
-   The AutoPilot software in this document is ArduPlane 4.0.5
-   The AutoPilot hardware in this instance is a CUAV v5 Nano and we will be connected to CAN1; any ArduPilot hardware with CAN broken out will be configured in functionally the same way

All of the hardware AutoPilot devices currently supported in ArduPilot do not provide suitable power to the CAN ports.  Because of this, it is required that you supply appropriate current to the UAVCAN servos outside of that bus.  Hitec has a collection of breakout boards to meet this demand and simplify wiring.  Do not forget to connect a bus termination resistor if you are not connecting any other devices.

After connecting to ArduPilot, the following parameter changes are required:

- :ref:`CAN_D1_PROTOCOL<CAN_D1_PROTOCOL>` =1
- :ref:`CAN_P1_DRIVER<CAN_P1_DRIVER>` =1

Once CAN_P1_DRIVER is changed from 0 to 1, you will need to reboot the autopilot gain access to the rest of the CAN Parameters:

``CAN_D1_PROTOCOL`` =1

``CAN_D1_UC_NODE`` =10

``CAN_D1_UC_SRV_BM`` =2

``CAN_D1_UC_SRV_RT`` =50

``CAN_P1_BITRATE`` =1000000

``CAN_P1_DRIVER`` =1

For ``CAN_D1_UC_SRV_BM`` you will need to know how to compute the bitmask for the servo you are using; This is an ArduPilot bitmask, not one for the servo itself.  It is telling ArduPilot to copy any PWM-OUT for the corresponding servos to the UAVCAN bus in the appropriate format.  Fortunately, Mission Planner makes this very easy with a pop-up once you click that value.  Since we’re wanting to make SERVO2 be UAVCAN, we’ll select that servo in this pop-up, close it, and press [Write Params]


Next verify ``SERVO2_FUNCTION`` is configured to be our Elevator servo by setting it to ``19`` if it isn’t already, and pressing [Write Params]
Optionally, we may want to command the servo at a rate higher than 50Hz.  Depending on which servo you have, you can configure ``CAN_D1_UC_SRV_RT`` to be a number greater than 50Hz. Note that setting this value to a number higher than your system's looptime will have no impact.

You can now proceed with the rest of your configuration and assuming power and configurations are all correct, be able to move the servo via RC Input or ground stabilization check (move the plane around in FBWA, for instance)

Log Analysis
============
Since you've configured the servo to stream data at 50Hz, ArduPilot will be able to see this data and will log it at the stream rate you’ve configured.  To view this data, open your favorite dataflash viewer, open the log, and browse to CSRV:
 

   .. image:: ../../../images/hitec-uavcan-servos-csrv.jpg


Here is an example of RCOU.C2 (SERVO2’s PWM Value) mapped against CSRV[2].Pos

   .. image:: ../../../images/hitec-uavcan-servos-ardulog1.png
       :width: 450px

Here is another sample where CSRV[2].Pos is plotted against CSRV[2].Force to show how randomly providing resistance to the servo while moving it using FBWA is logged:

   .. image:: ../../../images/hitec-uavcan-servos-ardulog2.png
       :width: 450px

