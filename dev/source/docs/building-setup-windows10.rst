
.. _building-setup-windows10:

===========================================================
Setting up the waf Build Environment on Windows10 using WSL
===========================================================

These setup instructions describe how to setup "Bash on Ubuntu on Windows" (aka "Windows Subsystem for Linux") which allows building with waf.  Alternative setup instructions for building with **make** are :ref:`here <building-setup-windows>`.

.. image:: ../images/build-on-windows10-top-image.jpg
    :target: ../_images/build-on-windows10-top-image.jpg

Setup steps
-----------

#. Enable Ubuntu on Windows which includes the following steps (original `How-To Geek's instructions here <http://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/>`__):

    - Under Settings >> Update & Security >> For developers, enable "Developer mode"

       .. image:: ../images/build-on-windows10-devmode.png
           :width: 70%
           :target: ../_images/build-on-windows10-devmode.png

   - Under Control Panel >> Programs >> Turn Windows features on or off, enable "Windows Subsystem for Linux

       .. image:: ../images/build-on-windows10-subsys-for-linux.png
           :width: 70%
           :target: ../_images/build-on-windows10-subsys-for-linux.png

   - Open the Microsoft Store (open a browser to http://aka.ms/wslstore) and install Ubuntu
   - Push the Launch button which will open a Bash terminal and ask for a username and password

       .. image:: ../images/build-on-windows10-usernamepwd.png
           :width: 70%
           :target: ../_images/build-on-windows10-usernamepwd.png
   

#. From withing the Ubuntu bash terminal, run the Tools/scripts/install-prereqs-ubuntu.sh script:

    - cd to the directory you :ref:`cloned <git-clone>` ardupilot into.  This will be the same as the windows path but with a ``/mnt/`` prefix, the colon (":") removed from the drive letter, and all backslashes converted to forward slashes.  For example if ardupilot was cloned to ``C:\Users\rmackay9\Documents\GitHub\ardupilot`` you would type ``cd /mnt/c/Users/rmackay9/Documents/GitHub/ardupilot``
    - cd to the Tools/scripts directory ``cd Tools/scripts``
    - run the script ``./install-prereqs-ubuntu.sh``.  You will be asked for your Ubuntu root password which is the password provided in step 1 (above).  Respond with "Y" when it asks if you wish to install a package.
    .. image:: ../images/build-on-windows10-prereqs.png
       :target: ../_images/build-on-windows10-prereqs.png
    - if you see a message, "not a valid git repository" this means the version of git on your Windows machine is different from the version on the Ubuntu subsystem.  You may need to remove the modules directory using ``rm -rf modules/*`` and :ref:`install a newer version of git <git-install>` in one of the two environments.  The git version can be checked by typing ``git --version``.

#. remove the /opt/gcc-arm-none-eabi directory:

    - ``sudo rm -rf /opt/gcc-arm-none-eabi-4_9-2015q3/bin``

#. Install some extra tools, so we can compile for the various targets:

   .. code-block:: python

       sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
       sudo apt-get update
       sudo apt-get install cmake

       sudo add-apt-repository ppa:ubuntu-toolchain-r/test
       sudo apt-get update
       sudo apt-get install g++-4.9

       sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded
       sudo apt-get update
       sudo apt-get install gcc-arm-none-eabi

       sudo apt-get update
       sudo apt-get upgrade

.. note::

    Bash can be configured to start from the directory you :ref:`cloned <git-clone>` ardupilot into by editing the .bashrc file and adding the "cd" command from Step 2 (above) to the end


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

   if you want to unhide Ubuntu system folder on Windows, open a command prompt as administrator,
   navigate to Ubuntu system path (C:\\Users\\username\\AppData\\Local) and change the folder attributes. So (where "username" is your user name):
   
   .. code-block:: python
   
       cd C:\Users\username\AppData\Local
       attrib -s -h lxss
       
   You can make a direct access to "lxss" folder on your desktop for a quick navigation.
