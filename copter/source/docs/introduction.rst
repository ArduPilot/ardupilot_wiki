.. _introduction:

===============================
Introducing Copter
===============================

Copter, an advanced and versatile open-source autopilot system, is specially designed for multicopters, helicopters, and other rotor vehicles. 
It provides a :ref:`wide variety of flight modes <flight-modes>` ranging from fully manual to completely autonomous operations.

.. image:: ../images/copter-introduction-diagram.jpg
    :target: ../_images/copter-introduction-diagram.jpg

Copter is a part of the broader ArduPilot software platform. 
It works with a wide array of :ref:`Ground Control Station programs <common-choosing-a-ground-station>`, enabling vehicle setup, real-time flight monitoring, and robust mission planning. 
As part of the larger ArduPilot ecosystem, Copter can be used with shared simulators, log analysis tools, and advanced APIs for vehicle control.

ArduPilot is the preferred platform for many commercially available autopilot systems and can significantly enhance DIY multirotor's capabilities.

Key Features
============

Copter boasts of several remarkable features:

-  High precision :ref:`acrobatic mode <acro-mode>`: Perform aggressive maneuvers including flips!
-  :ref:`Auto-level <stabilize-mode>` and :ref:`Altitude Hold <altholdmode>` modes: Fly straight and level effortlessly or use :ref:`simple <simpleandsuper-simple-modes>` mode to eliminate the need for the pilot to maintain the vehicle's heading. The autopilot cleverly interprets stick movements irrespective of the copter's orientation.
-  :ref:`Loiter <loiter-mode>` and :ref:`PosHold <poshold-mode>` modes: The vehicle autonomously maintains its position using GPS, accelerometers, and a barometer.
-  :ref:`Return to launch <rtl-mode>`: A single flip of a switch returns the Copter to its launch site, landing it automatically.
-  :ref:`Ad-hoc commands in Flight <ac2_guidedmode>`: If equipped with a two-way telemetry radio, simply click on the map, and the vehicle will fly to the selected location.
-  :ref:`Autonomous missions <auto-mode>`: Design complex missions with hundreds of GPS waypoints using a ground station. Switch the vehicle to "AUTO", and it autonomously takes off, completes the mission, returns home, lands, and disarms without any human intervention.
-  :ref:`Failsafes <failsafe-landing-page>`: The software continuously monitors the system's state and autonomously triggers a return-to-home if it loses contact with the pilot, encounters low battery, or strays outside a defined geofence.
-  **Flexibility and Customizability**: Copter can fly :ref:`all shapes and sizes of vehicles <common-all-vehicle-types>` according to your preferences as it allows access to hundreds of parameters controlling its behavior.
-  **No Vendor Lock-in**: ArduPilot is an entirely open-source platform backed by a diverse community of developers. It leaves you in total control of the software on your vehicle and its performance.

Getting Started
===============

To begin, you need a multicopter equipped with an :ref:`ArduPilot compatible autopilot <common-autopilots>`. 
You can choose from a list of :ref:`ready-to-fly vehicles <common-rtf>` or build your own.

If you opt for a :ref:`ready-to-fly vehicle <common-rtf>`, it should arrive pre-configured and tuned, ready for its maiden flight. 
Ensure to read the manufacturer's instructions, particularly safety-related sections, before flying. 
After :ref:`installing the ground station <common-install-gcs>`, you can move on to the :ref:`First Flight <flying-arducopter>` instructions.

.. warning:: 

   Whether using an RTF or DIY vehicle, autonomous vehicles can be
   hazardous! Always adhere to :ref:`best safety practices <safety-multicopter>` and heed all safety
   warnings.

If you plan to build your own multicopter, the following pages will guide you. 
Begin by understanding what a multicopter can do, and how to select a frame, autopilot board, and other essential components. 
Then proceed to :ref:`First Time Setup <initial-setup>` to assemble your copter and :ref:`First Flight <flying-arducopter>` to configure and tune it.

Discover More About Copter
==========================

To delve deeper into Copter and key configuration decisions, explore the topics below:

.. toctree::
    :maxdepth: 1

    How Multicopters Work <what-is-a-multicopter-and-how-does-it-work>
    Choosing a MultiCopter Frame <choosing-a-frame>
    Choosing an Autopilot <common-choosing-a-flight-controller>
    Choosing a Ground Station <common-choosing-a-ground-station>
    Building Your Own Frame <what-you-need>
    MultiCopter Safety <safety-multicopter>
    Ready to Fly Vehicles <common-rtf>
    Supported Vehicle Types <common-all-vehicle-types>
    Use-Case Overview <copter-use-case-overview>
