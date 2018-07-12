.. _troubleshooting-pixhawkpx4-boot:

============================
Troubleshooting Pixhawk Boot
============================

This article explains how to check if the Pixhawk has booted properly.

The following tests can help you determine if boot has failed, and possible causes:

-  Check the Pixhawk :ref:`LEDs <copter:common-leds-pixhawk>`
   and
   :ref:`Sounds <copter:common-sounds-pixhawkpx4>`
   as these can immediately confirm a successful boot. If boot fails,
   these can broadly indicate the point of failure.
-  Check that the the board has appropriate ArduPilot firwmare installed.
-  Ensure the memory card is fully inserted into the Pixhawk/controller card socket.
-  **Check the boot log:**

   #. Remove the SD card from from the board and insert into your
      regular computer
   #. open the **APM/boot.log** file and check that it contains the
      following lines

      ::

          Starting APM sensors
          Trying PX4IO board
          PX4IO board OK
          Starting ArduPilot
          rc.APM finished

   #. If the file does not exist or the contents are different from
      above then reinstall the Copter/Plane/Rover firmware.
