.. _install-mission-planner:

==========================
Installing Mission Planner
==========================

Mission Planner was designed for native Windows installation. However, it is possible to use it under Linux (with some occasional issues) and there is a Beta version for Android OS.

Windows Installation
====================

The following instructions show how to install *Mission Planner* on Windows. These instructions will be suitable for most users.  For advanced users and non-standard installations, instructions are found :ref:`here: <mission-planner-advanced-installation>`.  A useful video guide for advanced installation of *Mission Planner* is located `here <https://youtu.be/cw7adeD1yeg>`__.    

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
--------------------

Once installation is complete, open *Mission Planner* by clicking on its system icon.

Then you can either:

- :ref:`Connect Mission Planner to AutoPilot <common-connect-mission-planner-autopilot>` in order to receive telemetry and control the vehicle OR
- :ref:`Load Firmware <common-loading-firmware>` OR :ref:`plan autonomous missions <common-mission-planning>`.

Updating Mission Planner
------------------------

*Mission Planner* automatically notifies you about available updates (when it is connected to the Internet).

Please always run the most current version of Mission Planner.

.. toctree::
    :maxdepth: 1

    Mission Planner Advanced Installation <mission-planner-advanced-installation>

Mission Planner on Android
==========================

A version for Android is in development and can be downloaded from the `Google Play Store here <https://play.google.com/store/apps/details?id=com.michaeloborne.MissionPlanner>`__.

The latest version is also available `here <https://github.com/ArduPilot/MissionPlanner/releases/tag/latest>`__. Download to device, and double click on it to install.

Mission Planner on Linux
========================

Mission Planner does run on Linux, but will have occasional issues and/or crashes, and not all functions are available. QGC and MAVProxy are alternatives that run stably in Linux, but if Mission Planner is really desired, there are two ways to run it.

Linux development build
-----------------------

A Linux build of Mission Planner for x86_64 PCs is available. It needs the Mono runtime, but no SDK or compiler. On Ubuntu 22.04/24.04 or Debian:

- Download ``MissionPlanner-linux-x86_64.tar.gz`` and its ``.sha256`` file from the `Linux development release <https://github.com/ArduPilot/MissionPlanner/releases/tag/linux-latest>`__.
- Install the runtime packages, check the download, extract it and run it:

    ::

       sudo apt update
       sudo apt install mono-runtime ca-certificates-mono libgdiplus fonts-dejavu-core \
         libmono-system-windows-forms4.0-cil libmono-system-web-extensions4.0-cil \
         libmono-system-management4.0-cil libmono-system-servicemodel4.0a-cil \
         libmono-system-data-entity4.0-cil libmono-system-net-http4.0-cil \
         libgl1 libgl1-mesa-dri
       sha256sum -c MissionPlanner-linux-x86_64.tar.gz.sha256
       tar -xzf MissionPlanner-linux-x86_64.tar.gz
       ./MissionPlanner-linux-x86_64/run.sh

- To connect over USB or serial, your user needs access to the port, which usually means being in the ``dialout`` group (``sudo usermod -aG dialout $USER``, then log out and back in).
- To update, download and extract a new archive.

Other distributions need the equivalent Mono 6.8 or later packages. See the `Mission Planner Linux README <https://github.com/ArduPilot/MissionPlanner/blob/master/Linux/README.md>`__ for details and for building it from source.

Windows version under Mono
--------------------------

The Windows version of Mission Planner can also be run using Mono:

- Download and install the latest version of MONO from `here <https://www.mono-project.com/download/stable/>`__
- Download Mission Planner as a zip file from `here <https://firmware.ardupilot.org/Tools/MissionPlanner/MissionPlanner-latest.zip>`__ and  unzip to a directory.
- Change to that directory and execute:

    ::
    
       mono MissionPlanner.exe

.. note::

   **Zorin OS users:** If you see errors with .exe 
   files during build, run::

      sudo apt install mono-complete
