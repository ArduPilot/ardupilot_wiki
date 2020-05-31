.. _building-setup-linux:

===============================================
Setting up the Build Environment (Linux/Ubuntu)
===============================================

This article shows how to setup your build environment on Linux/Ubuntu machines.


Setup on Ubuntu
===============

Get git
-------

.. include:: git-install.rst
    :start-after: inclusion-marker-do-not-remove
    :end-before: Alternative for Windows user

Clone ArduPilot repository
--------------------------


.. include:: git-clone.rst
    :start-after: inclusion-marker-do-not-remove
    :end-before: Cloning with the GitHub GUI (Windows or MAC)


.. note:: in case some firewalls do not allow ssh access which can cause the above submodule updates to fail, in this instance you can tell git to unilaterally use https through the following command:

    ::

         git config --global url."https://" 

    to use https protocols instead of the default git:// prefix.



Install some required packages
------------------------------

If you are on a debian based system (such as Ubuntu or Mint), we provide `a script <https://github.com/ArduPilot/ardupilot/blob/master/Tools/environment_install/install-prereqs-ubuntu.sh>`__ that will do it for you. From ardupilot directory :
::

    Tools/environment_install/install-prereqs-ubuntu.sh -y

Reload the path (log-out and log-in to make permanent):

::

    . ~/.profile

Now you should be able to build with waf as described in `BUILD.md <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__.

.. note:: At this point you have already installed the MAVProxy Ground Control Station (MAVProxy GCS) and are also ready to do Software In the Loop (SITL) simulations of the vehicle code. See :ref:`sitl-simulator-software-in-the-loop`  and :ref:`setting-up-sitl-on-linux` . You are ready to not only build the code, but run your build in the Ardupilot SITL simulator.


.. youtube:: 4B8BVskH0vc

Cleaning
========

If there have been updates to some git submodules you may need to do a full clean build. To do that use:

::

    ./waf clean

that will remove the build artifacts so you can do a `build <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__ from scratch

--------------------

Setup for other Distributions Using the STM Toolchain
=====================================================



Add some directories to your search path (Facultative)
------------------------------------------------------

.. note::

    ONLY if you didn't run the install-prereqs script from previous step.

Add the following lines to the end of your ".bashrc" in your home
directory (notice the . on the start of that filename. Also, this is a
hidden file, so if you're using a file manager, make sure to turn on
"show hidden files").

::

    export PATH=$PATH:$HOME/ardupilot/Tools/autotest
    export PATH=/usr/lib/ccache:$PATH

Then reload your PATH by using the "dot" command in a terminal

::

    . ~/.bashrc
    
.. warning::

    Do not use this if you have already use the ``install-prereqs-ubuntu.sh`` script !


To build for a autopilot target on Linux you need the
following tools and git repositories:

-  The gcc-arm cross-compiler from `here <https://firmware.ardupilot.org/Tools/STM32-tools/>`__
   (ArduPilot is only built and tested on these specific versions of gcc-arm; if installed
   with ``apt-get`` gcc-arm will not produce a working binary in many cases)
-  gnu make, gawk and associated standard Linux build tools
-  On a 64 bit system you will also need to have installed libc6-i386.

Also, it's worth mentioning here that you want to ensure that the
modemmanager package is not installed and the modem-manager process is
not running.

Compiler
--------

You need the specific gcc-arm cross-compiler linked above. You need to unpack it where you want, for now let's call this location TARGET_DIR:

::

    cd TARGET_DIR
    tar -xjvf gcc-arm-none-eabi-6-2017-q2-update-linux.tar.bz2

and then add the bin directory from the tarball to your $PATH by editing
the $HOME/.bashrc file and adding a line like this to the end. TARGET_DIR is the location choose previouly where you unpack the toolchain:

``export PATH=$PATH:TARGET_DIR/gcc-arm-none-eabi-6-2017-q2/bin``

Permissions
-----------

You need to make your user a member of the dialout group:

::

    sudo usermod -a -G dialout $USER

You will need to log out and then log back in for the group change to take effect.



Now you should be able to build with waf as described in `BUILD.md <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__.

ccache for faster builds
========================

Installing *ccache* will speed up your builds a lot. Once you install it
(for example with "sudo apt-get install ccache") you should link the
compiler into /usr/lib/ccache like this:

::

    cd /usr/lib/ccache
    sudo ln -s /usr/bin/ccache arm-none-eabi-g++
    sudo ln -s /usr/bin/ccache arm-none-eabi-gcc

Then add /usr/lib/ccache to the front of your $PATH

---------

Additional Steps for macOS mojave
=================================
Due to some changes binutils installed via brew have stopped working for macOS mojave leading to crashing builds. So if installed, remove via following command:

::

    brew uninstall binutils

Also you will need to install the c++ include headers to /usr/include to do that. Run the following in commandline and follow the installation routine:

::

    open /Library/Developer/CommandLineTools/Packages/macOS_SDK_headers_for_macOS_10.14.pkg

---------

Setup using Docker
==================

Clone ArduPilot repository
--------------------------

.. include:: git-clone.rst
    :start-after: inclusion-marker-do-not-remove
    :end-before: Cloning with the GitHub GUI (Windows or MAC)

How to Build the Docker Image
-----------------------------

Build the docker image and tag it with the name ardupilot:
::

    docker build . -t ardupilot

Run ArduPilot Container
-----------------------
The following command runs the docker container, linking your current directory with
the ardupilot source, and launches an interactive shell inside the container. From here
you can build ardupilot:
::

    docker run --rm -it -v `pwd`:/ardupilot ardupilot:latest bash


