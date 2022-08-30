.. _initial-setup:

================
First Time Setup
================

.. image:: ../images/Quad_Color6.jpg
    :target: ../_images/Quad_Color6.jpg

First-time setup of the autopilot includes downloading and installing a Ground Control Station (GCS), 
mounting the autopilot to the frame,
connecting it to the receiver, power and motors, 
and then performing initial configuration and calibration.

.. note::

   This section assumes that you've already :ref:`chosen and built a frame <choosing-a-frame>` and have
   :ref:`selected your autopilot <common-choosing-a-flight-controller>`.

For more information on each of these tasks (sorted by autopilot
within the sections) see the topics below:

.. toctree::
    :maxdepth: 1

    Install Ground Station Software <common-install-gcs>
    Autopilot System Assembly <autopilot-assembly-instructions>
    Loading Firmware to boards with existing ArduPilot firmware <common-loading-firmware-onto-pixhawk>
    Loading Firmware to boards without existing ArduPilot firmware<common-loading-firmware-onto-chibios-only-boards>
    Connect Mission Planner to AutoPilot <common-connect-mission-planner-autopilot>
    Configuration <configuring-hardware>
    
    
.. tip::

    The next section (:ref:`First Flight <flying-arducopter>`)
    explains how to start flying using Copter, and further tuning and
    configuration required for new systems.