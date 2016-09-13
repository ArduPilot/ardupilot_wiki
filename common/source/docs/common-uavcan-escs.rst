.. _common-uavcan-escs:

===========
UAVCAN ESCs
===========

Copter, Plane and Rover support UAVCAN Electronic Speed Controllers
(ESCs) that allow two-way communication with the flight controller
enabling potentially easier setup and in-flight monitoring of ESC and
motor health.

.. note::

   UAVCAN ESCs are not yet readily available for purchase but should appear in late 2016

..  youtube:: https://youtu.be/LnUmYgAINBc
    :width: 100%

Connecting to the Pixhawk
=========================

.. image:: ../../../images/Pixhawk_UAVCAN_ESC.jpg
    :target: ../_images/Pixhawk_UAVCAN_ESC.jpg

One ESC (it does not matter which) should be connected to the Pixhawk's
CAN port using a 4-pin DF13 to 4-pin ClickMate adapter cable. Each
subsequent ESC should be connected to the previous using a 4-pin
ClickMate cable.  The final ESC should have a CAN bus terminator plugged
into one of it's 4-pin ClickMate ports.

The FTDI Cable connection is only required for the one-time set-up
discussed below.

ESC setup using CLI
===================

Each ESC must go through a one-time manual setup using an `FTDI cable <http://store.jdrones.com/cable_ftdi_6pin_5v_p/cblftdi5v6p.htm>`__
to define it's UAVCAN Node Id and motor number.  In future versions of
ardupilot this will be replaced with a setup procedure using the mission
planner (and other GCSs).

The steps required are:

-  Connect the FTDI cable to the ESC's "Developer port" as shown in the
   image above.
-  Power the ESCs with a battery
-  Use a terminal program such as
   `Putty <http://www.chiark.greenend.org.uk/~sgtatham/putty/download.html>`__
   to connect to the ESC using the FTDI cable's COM port, using serial
   at 115200 baud
-  press **Enter** into the terminal and a "ch>" prompt should appear
-  type ``cfg list`` to see a full list of parameters
-  on motor #1 (i.e. front-right on a quad) set the uavcan_node_id to
   "10" and uavcan_esc_index to "0" by typing:

   -  ``cfg set uavcan_node_id 10``
   -  ``cfg set uavcan_esc_index 0``
   -  ``cfg save``   (to save the configuration)

-  to test the motor moves:

   -  ``dc arm``  (to enable the output)
   -  ``dc 0.2`` (to spin motor at 20%)
   -  ``dc`` (to stop motor)

-  repeat for motors #2 (back left on a quad), #3, #4, etc with each
   motor having a node-id and index one higher than the previous (in
   fact the node-id doesn't matter as long as it's unique).

.. image:: ../../../images/ESC_cli_setup.png
    :target: ../_images/ESC_cli_setup.png

Additional information
======================

`Pixhawk ESC wiki page <https://pixhawk.org/modules/pixhawk_esc>`__

`Pixhawk ESC configuration wiki page <https://pixhawk.org/firmware/px4esc>`__

`Pixhawk ESC firmware on github <https://github.com/PX4/sapog>`__
