.. _common-kde-can-escs:

============
KDE CAN ESCs
============

.. image:: ../../../images/kde-can-esc.jpg

KDECAN ESCs are high-end ESCs that allow control and feedback using a custom CAN protocol

.. note::

    Please use Copter-4.1 (or higher), Plane-4.1 (or higher) or Rover-4.1 (or higher) with these ESCs because while earlier versions of ArduPilot supported them, there were issues with the range and motor ordering which made the setup difficult.

.. note:: KDE has released new firmware that offers the option of using standard DroneCAN protocol or their custom CAN protocol. The following page details setup using older firmware or newer firmware when the custom CAN protocol is selected. To setup for use as a standard DroneCAN ESC see the firmware update and setup instructions `here <https://cdn.shopify.com/s/files/1/0496/8205/files/DroneCAN_QuickStart_1.0.3.pdf>`__.

Where To Buy
------------

- `KDE-UAS125UVC-HE <https://www.kdedirect.com/collections/uas-multi-rotor-electronics/products/kde-uas125uvc-he>`__ and `KDE-UAS85UVC <https://www.kdedirect.com/collections/uas-multi-rotor-electronics/products/kde-uas85uvc>`__ can be purchased from `KDEDirect.com <https://www.kdedirect.com/collections/uas-multi-rotor-electronics>`__ (other KDE ESCs may also support CAN, check the images of the ESC, those with "CAN" written on the side should work).
- `KDECAN Wire Kit <https://www.kdedirect.com/collections/kdecan-bus-cables/products/kdecan-kit-jst-ghr-wire-kit>`__ is also required

Configuring the ESCs
--------------------

`KDE's instructions for connecting and configuring with ArduPilot are here <https://cdn.shopify.com/s/files/1/0496/8205/files/KDECAN_Pixhawk_QuickStart.pdf>`__

- Download, install and run the `KDE Direct Device Manager <https://www.kdedirect.com/collections/uas-multi-rotor-electronics/products/kde-dms?page=specs>`__
- Connect the first ESC to your PC with a USB cable and configure the device as shown below

  .. image:: ../../../images/kdecan-device-manager.png
      :target: ../_images/kdecan-device-manager.png
      :width: 400px

- Repeat the configuration of all other ESCs increasing the "KDECAN ESC ID" by one each time.  It is best to label the ESC with a number that is the ESC ID - 1. Somewhat confusingly the lowest ESC IDs is 02 but this will mirror the autopilot's SERVO1 output

Connecting the ESCs
-------------------

- The 4-pin cable connecting the first ESC to the autopilot's CAN1 port must have all pins swapped on one end (ideally on the end connecting to the first ESC).  Pin1 (red) should be swapped with Pin4.  Pin2 should be swapped with Pin3.  This modified cable should be used to connect the autopilot's CAN1 port to the first ESC
- The ESCs should be daisy chained together using the KDECAN Wire Kit.  The CAN terminator (the 4-pin connector with a black loop) should be connected to the last ESC in the chain

  .. image:: ../../../images/kde-can-esc-cube.jpg
      :target: ../_images/kde-can-esc-cube.jpg
      :width: 400px

.. warning::

    If using a CubeBlack autopilot, the CAN1 and CAN2 labels are reversed.  These instructions assume the ESCs are connected to the CAN1 port which is labelled "CAN2" on CubeBlack autopilots

Configuration
-------------

- Set :ref:`CAN_D1_PROTOCOL <CAN_D1_PROTOCOL>` = 8 (KDECAN)
- Set :ref:`CAN_P1_DRIVER <CAN_P1_DRIVER>` = 1 (First driver) to specify that the ESCs are connected to the CAN1 port
- Set :ref:`MOT_PWM_MIN <MOT_PWM_MIN>` = 1000 and :ref:`MOT_PWM_MAX <MOT_PWM_MAX>` = 2000 so ArduPilot uses an output range that matches the ESCs input range
- Set :ref:`KDE_NPOLE <KDE_NPOLE>` to the number of poles on the motor (Note it may be necessary to reboot the autopilot to cause this parameter to appear).  If you are not sure of the number, you can probably count them by looking closely into the top of the motor.  Even if set incorrectly the motors should spin normally because this number is only used to calculate the RPM reported to the ground station and stored in the onboard logs

Logging and Reporting
---------------------

KDECAN ESCs provide information back to the autopilot which is recorded in the autopilot's onboard log's ESCn messages and can be viewed in any :ref:`ArduPilot compatible log viewer <common-logs>`.  This information includes:

- RPM
- Voltage
- Current
- ESC Temperature
- Total Current

The RCOU messages are also written to the onboard logs which hold the requested output level sent to the ESCs expressed as a number from 1000 (meaning stopped) to 2000 (meaning full output).

This information can also be viewed in real-time using a ground station.  If using the Mission Planner go to the Flight Data screen's status tab and look for esc1_rpm.

.. image:: ../../../images/dshot-realtime-esc-telem-in-mp.jpg
    :target: ../_images/dshot-realtime-esc-telem-in-mp.jpg
    :width: 450px

.. note::

   Sending ESC data to the GCS requires using MAVLink2.  Please check the appropriate SERIALx_PROTOCOL parameter is 2 (where "x" is the serial port number used for the telemetry connection).
