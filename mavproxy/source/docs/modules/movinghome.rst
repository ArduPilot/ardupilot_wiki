.. _movinghome:

==========
Movinghome
==========

Allows changing vehicle's home position to new GCS location, if the GCS position moves. Requires NMEA GPS attached to serial port of GCS.

.. code:: bash

    module load movinghome

Usage
=====

movinghome <status|on|off|radius|device|baud>

- on: enable position updates as GCS moves. If GSC moves beyond threshold, vehicle home will be updated to current GCS position.
- off: disable home position updating
- radius: threshold in meters that GCS must move to update home in vehicle.
- status: status report
- baud: baudrate of serial port which is connected to NMEA GPS
- device: name of serial port which is connected NMEA GPS


