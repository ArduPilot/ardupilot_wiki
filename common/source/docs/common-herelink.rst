.. _common-herelink:
[copywiki destination="plane,copter,rover,blimp"]
========================================
Herelink RC/HD Video Transmission System
========================================

Herelink is an integrated remote controller, ground station and wireless digital transmission system which can be used with the Cube Autopilot and Ardupilot. Herelink allows RC control, HD video and telemetry data to be transmitted up to 20km between the ground station and air unit, the Herelink remote controller features custom Solex TX and QGC applications and both air unit and ground station feature an integrated 8 core SOC for custom application development. Herelink 1.1 also includes Ethernet connectivity from the air unit to gimbals,cameras, and autopilots in addition to HDMI, SBUS and UART based I/O.

.. image:: ../../../images/herelink.jpg
    :target: ../_images/herelink.jpg

Features
--------
* Android based smart device with 5.46 inch 1080P capacitive touch screen.
* Integrated digital transmission system simultaneously transmitting video, telemetry and control. 
* Custom Solex TX and QGC ground station software with MAVLink support for ArduPilot and PX4.
* Dual hall effect self centering gimbals with removable sticks. 
* 1080P HD video with dual digital video inputs. 
* Dual S.bus signal output for simultaneous control of autopilot and pan/tilt cameras.
* 6 customizable MAVLink buttons and Mavlink/Sbus hardware wheel. 
* RC gimbal control on dedicated service independent from Solex TX / QGC. 
* Tether and share video and telemetry via wifi ground station such as Mission Planner. 
* Onboard Arm Cortex A53 & Cortex A53 SOC in both air and ground units for custom applications. 


`Herelink System Overview <https://docs.cubepilot.org/user-guides/herelink/herelink-overview>`__

`Herelink User Manual <https://docs.cubepilot.org/user-guides/herelink/herelink-user-guides>`__

Updating QGC
------------

As of June 2024 the Herelink comes pre-installed with QGC-4.0.8.  If you wish to upgrade to QGC-4.4 (or higher) which includes improved camera and gimbal support follow these instructions:

1. Open the `QGC releases page <https://github.com/mavlink/qgroundcontrol/releases>`__
2. Click on the "Herelink" link and download the QGroundControl-Herelink.apk file
3. On the Herelink

  - Open settings by pulling down from the top of the screen and select the gear icon
  - Select "About Phone", scroll down and click "Build Number" multiple times to enable developer mode
  - Return to settings by clicking the back arrow button
  - Scroll down to "Developer Options" and enable "USB debugging"
  - Connect the Herelink to a PC via USB cable
  - Pull down from the top and select "USB charging this device" and change to "Use USB to Transfer files"

4. On the PC

  - Open a windows file explorer
  - Open the Herelink's filesystem (check under "This PC")
  - Drag-and-drop the Herelink-QGroundControl.apk file (downloaded above) to the Herelink's internal file system

5. On the Herelink

  - Pull down from the top and select "USB for file transfer" and restore "Use USB to" "Charge this device"
  - Open settings by pulling down from the top of the screen and select the gear icon
  - Select "Storage", "Internal shared storage", scroll down and select "Explore"
  - Click on "Herelink-QGroundControl.apk" and select "INSTALL"
  - To ease starting Herelink open the Herelink Launcher application, select the grid icon on the bottom right, click on "Herelink-QGroundControl" and select, "Add to Favorites"
  - When QGC is first started it will ask for various permission (select "Allow" for all), preferred units and Vehicle firmware (select "ArduPilot"

`This video <https://www.youtube.com/watch?v=a-cLzYD7HBk>`__ demonstrates how to update the QGC version in a general sense

`CubePilot instructions for installing a custom app <https://docs.cubepilot.org/user-guides/herelink/herelink-user-guides/installing-a-custom-app>`__ may also be useful


More Info on Herelink can be found on 

-   `Cubepilot <https://docs.cubepilot.org/user-guides/herelink/herelink-overview>`_
-   `Hex HereLink Website <http://www.proficnc.com/all-products/211-gps-module.html>`_
-   `Cubepilot Support Forum <https://discuss.cubepilot.org>`_