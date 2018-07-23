.. _mission-planner-advanced-installation:

=====================================
Mission Planner Advanced Installation
=====================================

This article explains how you can change where Mission Planner is
installed, and how to install older versions if needed (not
recommended).

Installing Mission Planner In a Different Location
==================================================

Sometimes you may want to keep  or install the current or different
version of Mission Planner in a folder different from the normal
location used by the :ref:`MSI installation <install-mission-planner>`. You can do this by
downloading the "zip" version which contains all the files necessary and
placing the files into a folder of your choice.

.. note::

   You will need a version of WinZip to install in a different
   location. Search for "Winzip free" for possible ad supported
   versions.

Download the most recent Mission Planner ZIP Files
--------------------------------------------------

You can find all the different versions of Mission Planner zip (and MSI)
files `here <http://firmware.ardupilot.org/Tools/MissionPlanner/>`__.
The most recent version of the zip file can be `downloaded from here <http://firmware.ardupilot.org/Tools/MissionPlanner/MissionPlanner-latest.zip>`__.

Download and save the zip file to your computer.

Install (Un-zip) the files to the desired folder
------------------------------------------------

Here is how you "install" a version into a different folder:

-  Unzip the downloaded zip files into the folder where you want them
   installed. (see your winzip manual)

   We suggest un-zipping to Program Files folder where the normal
   installation is, but with a different folder name I.E. a folder name
   like APM Planner-xxx where xxx is the version number.
-  After you do the un-zip, you should have all the files necessary to
   run that version of MP in the new folder.

   -  You can run this version by double clicking on the
      ArdupilotMegaPlanner10.exe file in the folder's root.
   -  Be sure to answer "no" to the upgrade notice if you want to stay
      with the version you are using in that folder.
   -  To create a desktop icon for that version do this:

      -  Find the fileArdupilotMegaPlanner10.exe in the folder's root.
      -  Right click and hold the file name and drag it to your desktop
      -  Select the 'Create a shortcut here" option when you release the
         mouse button.
      -  Right click on the new icon and select Rename.
      -  Change the name to include the version number or some text to
         identify the icon to start this version of MP

Using older version of Mission Planner
======================================

Using older versions of Mission Planner is discouraged because the
latest version should work with most platforms and will have fewer bugs.

However if you are using an older APM with an older version of firmware
that is working well for you it may make sense not to update the
firmware. Alternatively you might have more than one platform (for
instance a plane and a quad copter or two planes) and you want to keep a
different version of the Mission Planner for each on your ground
station.  This keeps the configuration files, the log files and saved
parameter files separate - making life much easier.

.. warning::

   Use older versions of Mission Planner at your own risk.  Do not
   attempt to upgrade firmware with older versions. Each new platform
   firmware update usually requires the latest Mission Planner
   version.
