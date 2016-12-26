.. _loading-the-firmware:

====================
Loading the Firmware
====================

The antenna tracker firmware can be loaded from the Mission Planner's
**Initial Setup \| Install Firmware** very :ref:`much like Copter. <planner:common-loading-firmware-onto-pixhawk>`

Use Mission Planner's Install Firmware screen to load the firmware
==================================================================

.. image:: ../images/AntennaTracker_MP_LoadFirmware.jpg
    :target: ../_images/AntennaTracker_MP_LoadFirmware.jpg

-  Connect the flight control board to the computer using the mini USB cable
-  Select the appropriate COM port on the top right of the screen but do
   not push the Connect button
-  From the **Initial Setup \| Install Firmware** page click on the
   "Antenna Tracker" icon on the bottom left.  Follow any instructions
   that appear and after a minute or so the firmware should be uploaded
   and you should be able to connect with the mission planner and see
   the Flight Data screen's HUD update.

.. note::

   Because the EEPROM format is different between the Antenna Tracker
   firmware and any flight controller software that was previously
   installed on the board (i.e. Copter or Plane) it is likely the EEPROM
   will have been erased by the firmware upload.

.. note::

   Firmware for other boards (APM1, Flymaple, etc) cannot be loaded
   directly from the mission planner but must be compiled and loaded onto
   the board more manually.
