.. _mission-planner-terminal:

==================================
Archived: Mission Planner Terminal
==================================

.. warning::

   The CLI is being phased out on all platforms. The features were
   massively cut down with AC3.1.5 and then it was removed for the APM2
   completely anyway with AC3.2. All CLI functions are now available via
   regular MAVLink commands in the GCSs. This article has been moved to the
   dev wiki and may be considered deprecated.

The Terminal tab is visible on the top icon row when "Advanced View" is
checked in the Config tab.

.. image:: ../images/Capture__2_.png
    :target: ../_images/Capture__2_.png

This section of Mission Planner, invoked by the Menu item Terminal at
the top of Mission Planner, opens a window that allows communication
with the auto pilot in a 'terminal' mode. This mode allows many actions
specific to the auto pilot and specific to the control firmware in the
autopilot. Copter, Plane and rovers will have different options. Refer
to the vehicle specific area of the website for details about your
specific platform.

Note:  When you open this menu item, the connection to your APM or PX4
will not be changed.  If connected, you will remain connected. This is
to prevent loss of control in case you are in a mission or your vehicle
is armed and in operation.  You must click the Connect APM or Connect
PX4 button to use the features in this screen.

.. image:: ../images/mpterminalAPM.jpg
    :target: ../_images/mpterminalAPM.jpg

Screen Actions:  There are several buttons in this screen.  Here is the
function of each:

-  Connect APM:  Reboot the APM in CLI mode and enable this screen.
    Caution, previous connection will be lost.
-  Connect PX4:  Reboot the PX4 in CLI mode and enable this screen.
    Caution, previous connection will be lost.
-  Show Settings: This simply lists all the the parameters in your auto
   pilot.  Better to use the Configuration Tuning section of Mission
   Planner
-  Setup Radio: Goes to the setup section, radio and allows you to set
   up the radio.  Better to use the Confiuration Tuning section of
   Mission Planner but sometimes this is useful.
-  Tests: Goes to the tests section of CLI and lists the options. You
   can type any of the options, press Enter to run the test.
-  Log Download: Opens a new screen for downloading and performing other
   actions related to the Flash Data logs that are recorded in the auto
   pilot. These logs are recorded when the APM or PX4 is enabled and
   stopped when it is disabled.  See the links below for more
   information.
-  Log Browse:  Lets you open a log file  (file with extension .log)
   that was previously saved and graph the various parameters that were
   recorded.  See the Data Flash Logs link below for more detail.

Here is where you will find other details:

:ref:`Flash Data Logs <planner:common-downloading-and-analyzing-data-logs-in-mission-planner>` (specific
to APM)  How to download and analyze  Data Flash logs.

:ref:`Sensor Testing <copter:common-sensor-testing>`:
(Specific to APM)

.. toctree::
    :maxdepth: 1

    Mission Planner CLI Setup Test modes <cli-setup-test-modes>
    Mission Planner Command Line Interface (CLI) <commonmission-planner-command-line-interface-cli>
    Setting up flight modes with the CLI <setting-up-flight-modes-with-the-cli>
    Using the Command Line Interface <using-the-command-line-interface>
    Using the Command Line Interface to Configure Copter <using-the-command-line-interpreter-to-configure-apmcopter>