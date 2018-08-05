
.. _building-setup-windows10:

===========================================================
Setting up the waf Build Environment on Windows10 using WSL
===========================================================

These setup instructions describe how to setup "Bash on Ubuntu on Windows" (aka "Windows Subsystem for Linux") which allows building with waf.  Alternative setup instructions for building with **make** are :ref:`here <building-setup-windows>`.

.. image:: ../images/build-on-windows10-top-image.jpg
    :target: ../_images/build-on-windows10-top-image.jpg

Setup steps
-----------

.. note::

Starting with the Fall Creators Update (Windows 10 version 1803), enabling the Developer Mode is not longer required for using WSL.

#. Enable Ubuntu on Windows which includes the following steps (original `How-To Geek's instructions here <http://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/>`__):

   - Under Control Panel >> Programs >> Turn Windows features on or off, enable "Windows Subsystem for Linux

       .. image:: ../images/build-on-windows10-subsys-for-linux.png
           :width: 70%
           :target: ../_images/build-on-windows10-subsys-for-linux.png

   - Open the Microsoft Store and install Ubuntu 16.04 (Direct link: https://www.microsoft.com/store/productId/9PJN388HP8C9)
   - Push the Launch button which will open a Bash terminal and ask for a username and password

       .. image:: ../images/build-on-windows10-usernamepwd.png
           :width: 70%
           :target: ../_images/build-on-windows10-usernamepwd.png
           
#. From withing the Ubuntu bash terminal, update your system to the latest packages:

    - ``sudo apt-get update``
    - ``sudo apt-get upgrade``
    
#. Copy the Ardupilot source files to your local Ubuntu filesystem by cloning the git repository and updating the submodules. (Accessing the source files on your Windows filesystem won't work because the must be kept on the native WSl filesystem)

    - ``git clone https://github.com/ardupilot/ardupilot.git``
    - ``cd ardupilot``
    - ``git submodule init``
    - ``git submodule update --recursive``

#.  run the Tools/scripts/install-prereqs-ubuntu.sh script:

    - run the script ``./Tools/scripts/install-prereqs-ubuntu.sh``.  You will be asked for your Ubuntu root password which is the password provided in step 1 (above).  Respond with "Y" when it asks if you wish to install a package.

    .. image:: ../images/build-on-windows10-prereqs.png
       :target: ../_images/build-on-windows10-prereqs.png
       
#. Remove arm-none-earbi-ar from the path in ~/.profile because the script installs the i386 version of the compiller, which does not run under a x64 WSL (unlike in real linux):

    - Open the file in a text editor ``nano ~/.profile``
    - Search for the line ``export PATH=/opt/gcc-arm-none-eabi-4_9-2015q3/bin:$PATH`` and delete it
    - Hit ``Ctrl + O`` to save the file

#. Install a 64 bit version of the compiler from the repositories:

    - ``apt-get install gcc-arm-none-eabi``

#. Reload the .profile script to let the changes take effect:

    - ``. ~/.profile``

Build with Waf
==============

You should now be able to start the "Ubuntu" application from your Windows Start menu and build with waf as described in `BUILD.md <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__.

   .. image:: ../images/build-on-windows10-configure.jpg
       :target: ../_images/build-on-windows10-configure.jpg

   .. image:: ../images/build-on-windows10-compile.jpg
        :target: ../_images/build-on-windows10-compile.jpg

.. note::

    WSL (Windows Subsystem Linux) does not have access to USB serial peripherals, so the waf --upload option
    does not work over USB.  A workaround is to upload using a ground station.  For example Mission Planner's Initial Setup >> Install Firmware screen has a "Load custom firmware" link on the bottom right.
    If this link is not visible, open Config/Tuning >> Planner and set the "Layout" to "Advanced".
    
    For network connected flight controllers, such as linux targets, --upload does function as described in `BUILD.md <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__
    
.. tip::

  You can run XWindows applications (including SITL) by installing VcXsrv on Windows and adding
  
  .. code-block:: python
  
      export DISPLAY=:0.0
      export LIBGL_ALWAYS_INDIRECT=1
      
  to your ~/.bashrc. For code editing you can install VSCode inside WSL.
  
.. tip::

   If you want to unhide Ubuntu system folder on Windows, open a command prompt as administrator,
   navigate to Ubuntu system path (C:\\Users\\username\\AppData\\Local) and change the folder attributes. So (where "username" is your user name):
   
   .. code-block:: python

       cd C:\Users\username\AppData\Local\Packages\CanonicalGroupLimited.UbuntuonWindows_79rhkp1fndgsc\LocalState
       attrib -s -h rootfs
       
   You can make a direct access to "lxss" folder on your desktop for a quick navigation.
