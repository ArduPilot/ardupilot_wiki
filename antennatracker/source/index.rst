.. _home:

===================
AntennaTracker Home
===================

.. tip::

    Keep up with the latest ArduPilot related blogs on `ArduPilot.org! <https://ardupilot.org/>`__

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

If your a developer wanting to work on AntennaTracker please join the
ArduPilot Discord chat channel which can be found in the Community menu
above.

If your a user looking for support on the AntennaTracker please go to
the support forums which can be found under the Community menu above.


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
    docs/common-logs
    How to Operate <docs/how-to-operate>
    docs/common-appendix
    docs/common-table-of-contents
    User Alerts <docs/common-user-alerts>
