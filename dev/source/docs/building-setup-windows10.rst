
.. _building-setup-windows10:

===========================================================
Setting up the waf Build Environment on Windows10 using WSL
===========================================================

These setup instructions describe how to setup "Bash on Ubuntu on Windows" (aka "Windows Subsystem for Linux") which allows building with waf.

.. image:: ../images/build-on-windows10-top-image.jpg
    :target: ../_images/build-on-windows10-top-image.jpg

Setup steps
-----------

.. note::

    Starting with the Fall Creators Update (Windows 10 version 1803), enabling the Developer Mode is not longer required for using WSL.

#. Enable Ubuntu on Windows which includes the following steps (original `How-To Geek's instructions here <http://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/>`__):

    .. note::

        In case of trouble, please refer to the official documentation from Microsoft : `<https://docs.microsoft.com/en-us/windows/wsl/install-win10>`__

   - Under Control Panel >> Programs >> Turn Windows features on or off, enable "Windows Subsystem for Linux" and restart your computer when asked.

       .. image:: ../images/build-on-windows10-subsys-for-linux.png
           :width: 70%
           :target: ../_images/build-on-windows10-subsys-for-linux.png

   - Open the Microsoft Store and install Ubuntu 18.04 (Direct link: https://www.microsoft.com/en-us/p/ubuntu-1804-lts/9n9tngvndl3q)
   - Push the Launch button which will open a Bash terminal and ask for a username and password

       .. image:: ../images/build-on-windows10-usernamepwd.png
           :width: 70%
           :target: ../_images/build-on-windows10-usernamepwd.png

    .. warning::

        Be careful not to lose your Linux password because it is often required, including when installing new packages. Also note Linux will not display any characters as you type your password

#. From withing the Ubuntu bash terminal, update your system to the latest packages:

    - ``sudo apt update && sudo apt upgrade && sudo apt full-upgrade``
    
#. Congratulations, you now have a working Ubuntu subsystem under Windows, you can now use our :ref:`Ubuntu instructions to install ArduPilot development environment <building-setup-linux>`

.. note::

    As of Windows 10 Build 16176 (April 2017), WSL (Windows Subsystem Linux) has support for access to USB serial peripherals. If you have not updated, please do so. That will allow you to compile and upload directly with the waf --upload option. You can also upload using a ground station.  For example Mission Planner's Initial Setup >> Install Firmware screen has a "Load custom firmware" link on the bottom right.
    If this link is not visible, open Config/Tuning >> Planner and set the "Layout" to "Advanced".
    
    For network connected flight controllers, such as linux targets, --upload does function as described in `BUILD.md <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__
    
.. tip::

  You can run XWindows applications (including SITL) by installing VcXsrv on Windows. For code editing you can install VSCode inside WSL.
  
.. tip::

   If you want to unhide Ubuntu system folder on Windows, open a command prompt as administrator,
   navigate to Ubuntu system path (C:\\Users\\username\\AppData\\Local) and change the folder attributes. So (where "username" is your user name):
   
   .. code-block:: python

       cd C:\Users\username\AppData\Local\Packages\CanonicalGroupLimited.UbuntuonWindows_79rhkp1fndgsc\LocalState
       attrib -s -h rootfs
       
   You can make a direct access to "lxss" folder on your desktop for a quick navigation.
   
.. tip::

    Windows 10 includes "Windows Defender" virus scanner. It will slow down WSL quite a bit. Disabling it greatly improves disk performance but increases your risk to viruses so disable at your own risk. Here is one of many resources/videos that show you how to disable it: https://www.youtube.com/watch?v=FmjblGay3AM
    
