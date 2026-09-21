.. _boat-configuration:

==================
Boat Configuration
==================

.. image:: ../images/boat-icon.png
    :target: ../_images/boat-icon.png

Boats can also be controlled with the standard Rover firmware.  To specify that the vehicle is a boat the :ref:`FRAME_CLASS <FRAME_CLASS>` parameter should be set to 2 (Boat).

The special features for Boats include:

- Boats appear as boats on the ground station

- :ref:`Sailboat support <sailboat-home>`

- :ref:`Loiter mode <loiter-mode>` for holding position

    - In :ref:`Auto <auto-mode>`, :ref:`Guided <guided-mode>`, :ref:`RTL <rtl-mode>` and :ref:`SmartRTL <smartrtl-mode>` modes the vehicle will attempt to maintain its position even after it reaches its destination

- :ref:`Sea-level GPS filtering <GPS_NAVFILTER>`

    - Not intended for use in water bodies at elevation (e.g. lakes/rivers on mountains)

- :ref:`Vectored Thrust <rover-vectored-thrust>` can be enabled to improve steering response on boats which use the steering servo to aim the motors

- Underwater-rated motors

    - :ref:`Blue Robotics thrusters <thrusters>`
    - :ref:`Torqeedo electric motors <common-torqeedo>`

- Underwater map generation

    - :ref:`Echosounders <common-underwater-sonars-landingpage>` for underwater mapping
    - :ref:`ReefMaster for bathymetry <reefmaster-for-bathymetry>`
