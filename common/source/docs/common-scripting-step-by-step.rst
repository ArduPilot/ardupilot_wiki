.. _common-scripting-step-by-step:
[copywiki destination="plane,copter,rover,dev"]

=============================
Script Setup and Use Examples
=============================

Testing a Script in SITL
========================
In Linux:

#. Assuming you have setup the build and simulation environment, run the simulation. See :ref:`using-sitl-for-ardupilot-testing`. For example:

.. code::

    sim_vehicle.py -v plane -f quadplane

Mavproxy ground control station will automatically be started. You can also start QGC (using the QGroundControl.AppImage) and/or Mission Planner (using WISE to run under Linux). Any or all three can be used simultaneously.

#. From your GCS, enable scripting by setting the :ref:`SCR_ENABLE<SCR_ENABLE>` parameter to "1". Restart the simulation.
#. A "scripts" sub-directory will be created in the directory you started the simulation, if it did not already exist. Place the script you downloaded into this directory. Again, restart the simulation.
#. If you get a "Scripting: out of memory" GCS message then you will need to increase :ref:`SCR_HEAP_SIZE<SCR_HEAP_SIZE>` parameter and try to restart the simulation.

In Mission Planner (Windows):

#. Start the simulation in Mission Planner as normal (see :ref:`mission-planner-simulation`)
#. In the CONFIG->Full Parameter List, set the :ref:`SCR_ENABLE<SCR_ENABLE>` parameter to "1". Restart the simulation.
#. The 'scripts' subdirectory will be located in your Documents->Mission Planner->sitl directory.Place the script you downloaded into this directory. Again, restart the simulation.
#. If you get a "Scripting: out of memory" GCS message then you will need to increase the :ref:`SCR_HEAP_SIZE<SCR_HEAP_SIZE>` parameter and try to restart the simulation.

Running a script on your Autopilot
==================================

#. Connect your GCS and navigate to its parameters screen (CONFIG->Full Parameter List in Mission Planner, Vehicle Setup->Parmeters in QGC) . Enable :ref:`SCR_ENABLE<SCR_ENABLE>` =1. Disconnect and reconnect the autopilot.
#. On your SD card, place the script in the APM/scripts directory. This directory will be created on the SD card when you have enabled the :ref:`SCR_ENABLE<SCR_ENABLE>` via your GCS. If it does not exist, create it and load the script. You can use Mission Planner's CONFIG->MAVFTP tab or take out the SD card and use a PC to write the script onto the card.
#. Restart the autopilot with the SD card in and the script(s) on it will load and execute.
#. If you get a "Scripting: out of memory" GCS message then you will need to increase :ref:`SCR_HEAP_SIZE<SCR_HEAP_SIZE>` parameter and try to restart the simulation. How much memory is needed is dependent on the script and the configuration of peripherals and features in use on the autopilot.

Once Script is running
======================

Many scripts will require parameters created by the script to be setup or RC switches configured. This should be explained in the script's README.md file co-located with the script in the ArduPilot repo.
