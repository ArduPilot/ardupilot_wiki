.. _home:

===================
AntennaTracker Home
===================

.. tip::

    The ArduPilot Developer Ecosystem is Evolving! 
    `Find out more here … <http://diydrones.com/profiles/blogs/a-new-chapter-in-ardupilot-development>`_

The *AntennaTracker* Project delivers firmware which allows you to use a
supported flight controller board (Pixhawk, APM2, etc.) as the
controller for an Antenna Tracker.

The tracker calculates the position of a remote vehicle using its own
GPS position and GPS telemetry from a vehicle running Copter, Rover or
Plane. It then uses this information to aim a directional antenna at
the vehicle.

Using a correctly aligned directional antenna *significantly improves* 
the range over which signals can be both sent and received from
a ground station.

..  youtube:: 8GCqYTDYZaM
    :width: 100%

Below is `Canberra UAV's tracker used during the 2014 OutBack Challenge <http://www.suasnews.com/outback-challenge-2014/>`__.

.. image:: /images/home_antennatracker.jpg
    :target: _images/home_antennatracker.jpg

This manual will guide you through setup and configuration process.

.. toctree::
    :hidden:
     
    Choosing an Antenna <docs/choosing-antenna>
    Frame Assembly <docs/frame-assembly>
    Loading the Firmware <docs/loading-the-firmware>
    Configuration <docs/configuration>
    docs/parameters
    Connecting with GCS <docs/connecting-with-gcs>
    How to Operate <docs/how-to-operate>
    docs/common-appendix
    docs/common-table-of-contents
