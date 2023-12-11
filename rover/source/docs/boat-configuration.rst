.. _boat-configuration:

==================
Boat Configuration
==================

.. image:: ../images/boat-icon.png
    :target: ../_images/boat-icon.png

Boats can also be controlled with the standard Rover firmware.  To specify that the vehicle is a boat the :ref:`FRAME_CLASS <FRAME_CLASS>` parameter should be set to 2 (Boat).

The special features for Boats include:

- Boats appear as boats on the ground station
- In :ref:`Auto <auto-mode>`, :ref:`Guided <guided-mode>`, :ref:`RTL <rtl-mode>` and :ref:`SmartRTL <smartrtl-mode>` modes the vehicle will attempt to maintain its position even after it reaches its destination
- :ref:`Thrusters <thrusters>`
- :ref:`Echosounders <common-underwater-sonars-landingpage>` for underwater mapping
- :ref:`Loiter mode <loiter-mode>` for holding position
- :ref:`ReefMaster for bathymetry <reefmaster-for-bathymetry>`
- :ref:`Torqeedo electric motors <common-torqeedo>`
- :ref:`Vectored Thrust <rover-vectored-thrust>` can be enabled to improve steering response on boats which use the steering servo to aim the motors
