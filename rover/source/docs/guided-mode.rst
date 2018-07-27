.. _guided-mode:

===========
Guided mode
===========

Guided mode is designed to allow ground stations or :ref:`companion computers <common-companion-computers>` to control the vehicle.

Sending Commands from the Ground Station
----------------------------------------

Most ground stations support commanding the vehicle to drive to a location specified by clicking on the map.  If using the Mission Planner:

- connect to the vehicle with a :ref:`telemetry radio <common-telemetry-landingpage>`
- from the Flight Data screen, right-mouse-button click on the map and select "Fly To Here".
- the vehicle will switch to Guided mode and attempted to drive to the location specified.

.. image:: ../images/rover-guided-fly-to-here.png
    :target: ../_images/rover-guided-fly-to-here.png

The ground station will normally set the vehicle to Guided mode before the target destination is sent meaning it is not necessary to set up Guided mode on the :ref:`transmitter mode switch <common-rc-transmitter-flight-mode-configuration>`.

Other controls
--------------

These additional mavlink messages are supported in Guided mode.  These are listed mostly for developers of ground stations or :ref:`companion computers <common-companion-computers>` applications:

-  `SET_ATTITUDE_TARGET <http://mavlink.org/messages/common#SET_ATTITUDE_TARGET>`__
-  `SET_POSITION_TARGET_LOCAL_NED <http://mavlink.org/messages/common#SET_POSITION_TARGET_LOCAL_NED>`__
-  `SET_POSITION_TARGET_GLOBAL_INT <http://mavlink.org/messages/common#SET_POSITION_TARGET_GLOBAL_INT>`__
-  ``MAV_CMD_NAV_SET_YAW_SPEED`` commands within a `COMMAND_LONG <http://mavlink.org/messages/common#COMMAND_LONG>`__




