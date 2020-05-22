.. _install-mission-planner:

====================================
Installing Mission Planner (Windows)
====================================

The below instructions show how to install *Mission Planner* on Windows. These instructions will be suitable for most users.  For advanced users and non-standard installations, instructions are found :ref:`here: <mission-planner-advanced-installation>`.  A useful video guide for advanced installation of *Mission Planner* is located `here <https://youtu.be/cw7adeD1yeg>`__.    

- Download the `latest Mission Planner installer from here <https://firmware.ardupilot.org/Tools/MissionPlanner/MissionPlanner-latest.msi>`__
- Double click on the downloaded **.msi** file to run the installer

  .. image:: ../images/mp_installation.png
      :target: _images/mp_installation.png

- Follow the instructions to complete the setup process. 
  The installation utility will automatically install any necessary software drivers. 
  If you receive a DirectX installation error, 
  please update your DirectX plug-in from the `Windows Download Center <https://www.microsoft.com/en-us/download/windows.aspx>`__.

- If you receive the warning pictured below, select **Install this driver software anyway** to continue.

  .. image:: ../../../images/driver_installation_warning.png
      :target: ../_images/driver_installation_warning.png

Mission Planner is normally installed in the **C:\\Program Files (x86)\\Mission Planner** folder.

An icon to open the *Mission Planner* is created according to your instructions during the installation.

Open Mission Planner
====================

Once installation is complete, open *Mission Planner* by clicking on its system icon.

Then you can either:

- :ref:`Connect Mission Planner to AutoPilot <common-connect-mission-planner-autopilot>` in order to receive telemetry and control the vehicle OR
- :ref:`Load Firmware <common-loading-firmware-onto-pixhawk>` OR :ref:`plan autonomous missions <common-mission-planning>` .

.. note:: If ArduPilot firmware is not already installed on the autopilot, see :ref:`Loading Firmware to boards without existing ArduPilot firmware <common-loading-firmware-onto-chibios-only-boards>`.

Updating Mission Planner
========================

*Mission Planner* automatically notifies you about available updates (when it is connected to the Internet).

Please always run the most current version of Mission Planner.

.. toctree::
    :maxdepth: 1

    Mission Planner Advanced Installation <mission-planner-advanced-installation>
