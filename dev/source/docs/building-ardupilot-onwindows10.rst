
.. _building-ardupilot-onwindows10:

=======================================
Building ArduPilot on Windows with Bash
=======================================
.. warning::
    This build method is EXPERIMENTAL for windows, although it can work well for some users, you may prefer to look at the more tested/proven windows build process here:  http://ardupilot.org/dev/docs/building-px4-with-make.html

.. image:: ../images/Windows10_Ubuntu_0.jpg
    :target: ../_images/Windows10_Ubuntu_0.jpg

This article shows how to build ArduPilot on Windows10, using the new "Bash on Ubuntu on Windows".

Since Windows 10 has Ubuntu as his new partner, it is possible to compile Ardupilot (apm1/apm2, Pixhawk/PX4, SITL...) in Windows without adding any strange thing: just install "Bash on Ubuntu on Windows" and follow next tutorial.
   
.. warning::

   This tutorial is intended to be a "copy & paste" approach. Just follow text step by step and be patient with yourself.

   "Bash on Ubuntu on Windows" only works under Windows10 x64.
   
   Copter 3.3 firmware (and later) and builds after Plane 3.4.0 no longer
   fit on APM boards. Plane, Rover and AntennaTracker builds can still be
   installed at time of writing but you can no longer build APM1/APM2.x off the
   master branch (you will need to build off a supported release branch).

.. tip::

   For install "Bash on Ubuntu on Windows" there are many tutorials on the web. An example is here:
   <http://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/>`__.


Setup Ardupilot Dev Enviromment for Ubuntu bash on Windows 10
===============

#. First, you have to insert your local host to the /etc/hosts file:

   .. code-block:: python
   
       sudo nano /etc/hosts

#. Insert this below first line:

   .. code-block:: python
   
       127.0.0.1 "hostname"
         
   (where "hostname" is your PC name)

#. Then, take root control over the bash:

   .. code-block:: python
   
       sudo passwd root
       su root

#. Back to main directory:

   .. code-block:: python
   
       cd

#. Install git:

   .. code-block:: python
   
       sudo apt-get install git

#. Install Arduino Core (for APM1/APM2 support):

   .. code-block:: python
   
       sudo apt-get install make gawk arduino-core

#. Make a new folder for future reference:

   .. code-block:: python
   
       mkdir GitHub
       cd GitHub

#. Clone ardupilot git:

   .. code-block:: python
   
       git clone https://github.com/ArduPilot/ardupilot.git

#. Run the install-prereqs-ubuntu.sh script:

   .. code-block:: python
   
       ./ardupilot/Tools/scripts/install-prereqs-ubuntu.sh

   .. image:: ../images/Windows10_Ubuntu_1.jpg
       :target: ../_images/Windows10_Ubuntu_1.jpg
       
#. At this point, we need to install some extra tools, because for now, Ubuntu bash on Windows only supports native x64 programs:

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

#. Now you have a nice compiler under Windows 10 thanks to Ubuntu.Next step is to prove it!

Compile ArduPilot Code
======================

#. For apm1/apm2 support is necesary to make a "config.mk" file (for compile ArduCopter 3.2.1), so navigate to main ardupilot git folder:

   .. code-block:: python
   
       cd
       cd GitHub/ardupilot

#. Is necesary to select correct branch (last supported release branch):

   .. code-block:: python
   
       git checkout ArduCopter-3.2.1
       cd ArduCopter
       make configure

#. If everything is ok, compile the code:

   .. code-block:: python
   
       cd ArduCopter
       make apm1

#. Or:

   .. code-block:: python
   
       make apm2

#. Otherwise, if you want to compile last ArduPlane compatible code whith apm1/2 (ArduPlane 3.4), you need to preserve above commented "config.mk" file and change your git branch:

   .. code-block:: python
   
       cd
       cd GitHub/ardupilot
       git checkout ArduPlane-release-AVR

#. If everything is ok, compile the code:

   .. code-block:: python
   
       cd ArduPlane
       make apm1

#. Or:

   .. code-block:: python
   
       make apm2

======================

#. If you want to compile Pixhawk/PX4 code, "config.mk" file is not necesary. So if you have compiled apm1/apm2 code, you have to remove it:

   .. code-block:: python
   
       cd
       cd GitHub/ardupilot
       rm config.mk

#. Now, select correct branch (for example, master branch):

   .. code-block:: python
   
       cd
       cd GitHub/ardupilot
       git checkout master

#. Navigate to your desired Ardupilot vehicle folder, in this case (complete path):

   .. code-block:: python
   
       cd
       cd GitHub/ardupilot/vehicle
       
   (where "vehicle" can be ArduCopter, ArduPlane, ArduRover,...)
         
#. In your vehicle folder, type "make" and press enter for see supported targets for compile. Just type "make" and your target for compile it, in this case:

   .. code-block:: python
   
       vehicle
       make board
       
   (where "board" can be px4-v2, navio, pxf,... Must see supported targets as mentioned above)
         
#. If everything is ok, you will see a message like this (depends of your target): PX4 ArduCopter Firmware is in ArduCopter-v2.px4

   .. image:: ../images/Windows10_Ubuntu_2.jpg
       :target: ../_images/Windows10_Ubuntu_2.jpg

======================

#. The new method of compilation "Waf" is also supported. Go to git ardupilot software:

   .. code-block:: python
   
       cd
       cd GitHub/ardupilot/

#. It's possible to get a list of supported boards on ArduPilot with the command below:

   .. code-block:: python
   
       ./waf list_boards
   
#. Configure your board:

   .. code-block:: python
   
       ./waf configure --board target
       
   (where "target" can be px4-v2, navio, pxf,... Must see supported boards as mentioned above)

   .. image:: ../images/Windows10_Ubuntu_3.jpg
       :target: ../_images/Windows10_Ubuntu_3.jpg
   
#. Compile for your target:

   .. code-block:: python
   
       ./waf vehicule
   
   (where "vehicle" can be copter, plane, rover,...)

   .. image:: ../images/Windows10_Ubuntu_4.jpg
       :target: ../_images/Windows10_Ubuntu_4.jpg

#. And here you are! You have compiled your code!

.. tip::

   if you want to unhide Ubuntu system folder on Windwos, open a command prompt as administrator,
   navigate to Ubuntu system path (C:\Users\username\AppData\Local) and change the folder attributes. So (where "username" is your user name):
   
   .. code-block:: python
   
       cd C:\Users\username\AppData\Local
       attrib -s -h lxss
       
   You can make a direct access to "lxss" folder on your desktop for a quick navigation.
