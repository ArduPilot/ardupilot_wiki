.. _installation-for-linux:

================================
Installing APM Planner for Linux
================================

System Requirements
===================

-  Ubuntu 14.04 or equivalent
-  300 MB free space
-  Internet connection to use maps

Download
--------

Download the latest deb file for your machine (from
`firmware.ardupilot.org <http://firmware.ardupilot.org/Tools/APMPlanner/>`__):

Debian (outdated)
+++++++++
-  `apm_planner2_latest_debian.deb <http://firmware.ardupilot.org/Tools/APMPlanner/apm_planner2_latest_debian.deb>`__
-  `apm_planner2_latest_debian64.deb <http://firmware.ardupilot.org/Tools/APMPlanner/apm_planner2_latest_debian64.deb>`__

Ubuntu Trusty (14.04)
+++++++++
-  `apm_planner2_latest_ubuntu_trusty.deb <http://firmware.ardupilot.org/Tools/APMPlanner/apm_planner2_latest_ubuntu_trusty.deb>`__
-  `apm_planner2_latest_ubuntu_trusty64.deb <http://firmware.ardupilot.org/Tools/APMPlanner/apm_planner2_latest_ubuntu_trusty64.deb>`__

Ubuntu Wily (15.10)
+++++++++
-  `apm_planner2_latest_ubuntu_wily64.deb <http://firmware.ardupilot.org/Tools/APMPlanner/apm_planner2_latest_ubuntu_wily64.deb>`__

Ubuntu Lastest (outdated)
+++++++++
-  `apm_planner2_latest_ubuntu.deb <http://firmware.ardupilot.org/Tools/APMPlanner/apm_planner2_latest_ubuntu.deb>`__
-  `apm_planner2_latest_ubuntu64.deb <http://firmware.ardupilot.org/Tools/APMPlanner/apm_planner2_latest_ubuntu64.deb>`__

Install Packages
================

Open a terminal window and go to the location where you downloaded the
.deb file from step 2 and type the following command:

::

    sudo dpkg -i apmplanner*.deb

The installation will likely fail because of missing dependencies. 
These dependencies can be installed with this command:

::

    sudo apt-get -f install

Then retry the apmplanner installation again:

::

    sudo dpkg -i apmplanner*.deb

Create a desktop icon
=====================

Click on `this link to download the apmplanner2.desktop <https://raw.githubusercontent.com/diydrones/apm_planner/master/debian/apmplanner2.desktop>`__
file from github.  Set the properties on the file to "Allow executing
file as program", then drag and drop the file to your start bar.

.. image:: ../images/apm_planner_2_install_linux_desktop_icon.jpg
    :target: ../_images/apm_planner_2_install_linux_desktop_icon.jpg

Click on the new Icon and the APM Planner 2 should start.

Common problems
===============

If you find the APM Planner 2 is unable to connect to the APM2/Pixhawk,
it may be because of the modemmanager package.  Uninstall this package
by entering the following command into a terminal window:

::

    sudo apt-get --purge remove modemmanager
