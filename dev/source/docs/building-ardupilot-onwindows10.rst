
.. _building-ardupilot-onwindows10:

===============================================
Building ArduPilot on Windows with waf and Bash
===============================================
.. warning::
    This build method is ONLY for Windows10 x64 and does not describe building for 
    deprecated Arduino based flight controllers (APM1.x, 2.x). It supports most users,
    however you may need or prefer to use the alternate windows build process here:
    http://ardupilot.org/dev/docs/building-px4-with-make.html

.. image:: ../images/Windows10_Ubuntu_0.jpg
    :target: ../_images/Windows10_Ubuntu_0.jpg

Microsoft has introduced WSL - the Windows Subsystem for Linux - into Windows 10.
This tutorial describes "Bash on Ubuntu on Windows", as it is currently the only released
distribution supported by WSL.  Other Linux distributions, such as Fedora, are likely to be
available in the future however will require slightly different setup to compile Ardupilot.

.. warning::

   This tutorial is intended to be a "copy & paste" guide. Just follow the process step by step and be patient with yourself!

.. tip::

   To install "Bash on Ubuntu on Windows" there are many tutorials on the web. An example is here:
   <http://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/>`__.


.. tip::
    With the latest "Fall Creators update" someting is broken in the file access from WSL to Windows directories.
    It seems that if you put ardupilot source code to a windows drive and try to build it from WSL via the /mnt/<drive> folders,
    the build will fail with random include errors. To make it work, you have to put your source code to a directory under the 
    "native" WSL filesystem or keep it on windows, share it, and mount via the drvfs driver, like this:

    .. code-block:: python
 
        mount -t drvfs '\\127.0.0.1\ardupilot' /devel/ardupilot


Setup Ardupilot Dev Enviromment for Ubuntu bash on Windows 10
=============================================================

#. First, you have to insert your local host to the /etc/hosts file:

   .. code-block:: python
   
       sudo nano /etc/hosts

#. Insert this below first line:

   .. code-block:: python
   
       127.0.0.1 "hostname"
         
   (where "hostname" is your PC name)

#. Then, take root control over the terminal:

   .. code-block:: python
   
       sudo passwd root
       su root

#. Back to main directory:

   .. code-block:: python
   
       cd

#. Install git:

   .. code-block:: python
   
       sudo apt-get install git

#. Make a new folder for future reference (optional):

   .. code-block:: python
   
       mkdir GitHub
       cd GitHub

#. Clone ardupilot git:

   .. code-block:: python
   
       git clone https://github.com/ArduPilot/ardupilot.git

#. Create a folder named "opt" under the root path while WSL doesn't have:

   .. code-block:: python
       
       cd /
       mkdir opt
       
#. Run the install-prereqs-ubuntu.sh script:

   .. code-block:: python
   
       ./ardupilot/Tools/scripts/install-prereqs-ubuntu.sh

   .. image:: ../images/Windows10_Ubuntu_1.jpg
       :target: ../_images/Windows10_Ubuntu_1.jpg
       
#. We also need to install some extra tools, so we can compile for the various Ardupilot targets:

   .. code-block:: python
   
       cd
       
       sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
       sudo apt-get update
       sudo apt-get install cmake
       
       sudo add-apt-repository ppa:ubuntu-toolchain-r/test
       sudo apt-get update
       sudo apt-get install g++-4.9
       
       sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded
       sudo apt-get update
       sudo apt-get install gcc-arm-none-eabi=4.9.3.2015q3-1trusty1
       
       sudo apt-get update
       sudo apt-get upgrade

Compile ArduPilot Code
======================

#. "Waf" is the preferred build tool for Ardupilot, and works well in Windows with Ubuntu Bash. Go to git ardupilot software:

   .. code-block:: python
   
       cd
       cd GitHub/ardupilot/

#. It's possible to get a list of supported boards on ArduPilot with the command below:

   .. code-block:: python
   
       ./waf list_boards
   
#. Configure your board:

   .. code-block:: python
   
       ./waf configure --board target
       
   (where "target" can be px4-v2, navio, pxf,... choose from supported boards as mentioned above)

   .. image:: ../images/Windows10_Ubuntu_3.jpg
       :target: ../_images/Windows10_Ubuntu_3.jpg
   
#. Compile for your target:

   .. code-block:: python
   
       ./waf vehicle
   
   (where "vehicle" can be copter, plane, rover,...)

   .. image:: ../images/Windows10_Ubuntu_4.jpg
       :target: ../_images/Windows10_Ubuntu_4.jpg

#. And here you are! You have compiled your code!

.. note::

    The released WSL does not have access to USB serial peripherals, so the --upload option
    in waf will not work over USB.  The compiled binary must be uploaded to your flight controller using another 
    tool, such as MissionPlanner as a "Custom Firmware".
    If you wish to upload via waf, at time of writing USB serial access is only available in the Windows Insider
    releases of Windows Subsystem for Linux (WSL), which requires you to join the Windows Insider program.
    https://insider.windows.com/GettingStarted
    For network connected flight controllers, such as linux targets, --upload does function as described here:
    https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md

.. tip::

   if you want to unhide Ubuntu system folder on Windows, open a command prompt as administrator,
   navigate to Ubuntu system path (C:\Users\username\AppData\Local) and change the folder attributes. So (where "username" is your user name):
   
   .. code-block:: python
   
       cd C:\Users\username\AppData\Local
       attrib -s -h lxss
       
   You can make a direct access to "lxss" folder on your desktop for a quick navigation.
   
A few things that you may need if things do not go to plan
==========================================================

   .. code-block:: python
   
       sudo apt-get install python
       git submodule update --recursive --init modules/uavcan
       pip install future
       pip install lxml
       sudo apt-get install genromfs
       sudo apt-get install unzip
       sudo apt-get install zip
       apt-get install python-empy
   
   (.............)
