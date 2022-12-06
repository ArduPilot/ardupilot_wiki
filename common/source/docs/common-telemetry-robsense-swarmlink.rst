.. _common-telemetry-robsense-swarmlink:
[copywiki destination="plane,copter,rover,blimp"]
==================
Robsense SwarmLink
==================

  .. image:: ../../../images/telemetry-robsense-swarmlink.png
	 :target: ../_images/telemetry-robsense-swarmlink.png

`Robsense SwarmLink <https://home.robsense.com/?page_id=862&lang=en>`__ telemetry radios allows connecting multiple drones to a single ground station without the need for multiple radios on the ground station side (i.e. it creates a mesh network).  Network monitoring and configuration software is also included.

.. warning::

   The developer team has been unable to contact Robsense leading us to believe the manufacturer is no longer supporting this product

Specifications (according to the manufacturer)
----------------------------------------------

- MCU: ARM Cortex-M3
- Frequency: 433Mhz
- Channel Bandwidth: 500Khz
- Modulation: LoRa Spread Spectrum
- TX Power: 20dBm
- Receive Sensitivity: -148dBm
- Voltage: 5V
- Current: 40mA
- Interfaces: Serial, MicroUSB, LAN
- Size: 83mm x 60mm x 20mm
- Weight: 65g

More details can be found `here <https://home.robsense.com/?page_id=862&lang=en#>`__.

EasySwarm
---------

Built on top of the SwarmLink hardware, `EasySwarm <https://guide.robsense.com/chapter3-1/communication/easyswarm.html>`__ is a developer focused ground station and development kit aimed at making swarming easier.  This free and open source software can be found in the `RobSense SwarmLink github repo <https://github.com/RobSenseTech/SwarmLink>`__.

Features include customised swarming policies, dynamic waypoint planning and real-time tracking of all vehicles.

  .. image:: ../../../images/telemetry-robsense-easyswarm-gcs.png
	 :target: ../_images/telemetry-robsense-easyswarm-gcs.png 

..  youtube:: 2qy5eZDyAp4
    :width: 100%
