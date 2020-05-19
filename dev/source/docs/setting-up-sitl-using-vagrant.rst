.. _setting-up-sitl-using-vagrant:

=============================
Setting up SITL using Vagrant
=============================

This article explains how to set up the :ref:`SITL ArduPilot Simulator <sitl-simulator-software-in-the-loop>` in a virtual machine
environment using `Vagrant <https://www.vagrantup.com/>`__, and connect
it to a Ground Control Station running on the host computer. This
approach is much easier and faster than
:ref:`manually <setting-up-sitl-on-windows>`
setting up a virtual machine to run SITL on Mac OSX or Windows (or
Linux).

These instructions have been tested on Windows 8.1.

Overview
========

The SITL (Software In The Loop) simulator allows you to run Plane,
Copter or Rover without any hardware. The simulator runs the normal
ArduPilot code as a native executable on a Linux PC. SITL can also be
run within a virtual machine on Windows, Mac OSX or Linux.

Vagrant is a tool for automating setting up and configuring development
environments running in virtual machines. While it is possible to
:ref:`manually set up SITL to run in a VM on Windows <setting-up-sitl-on-windows>`
(or Mac OSX), it is **much easier** (and more reproducible) to use
Vagrant to do this work for you.

.. note::

   Due to the way submodules are currently handled in the build
   system, it is not possible to have a repository which can be built
   on both the host and virtual machines.  A dedicated repository
   should be used for running the Vagrant virtual machine.

Preconditions
=============

-  *Git* (**1.8.x or later**) must be installed on the host computer.

   -  `Git for Windows <https://msysgit.github.io/>`__ (1.9.5) is
      recommended.

      .. note::

         The current windows px4 toolchain (v14) does not have a
               recent enough version of GIT

.. warning::

   You must use the newer
         version for the git submodule init step. After that setp you can
         use an older version.

   -  Ensure that *git* is set to leave line endings untouched. Click on
      your new “Git Shell (or Bash)” Icon (the terminal was installed
      when you installed git) and type in the following in the Git
      “MINGW32″ Terminal window:

      ::

          git config --global core.autocrlf false

-  *SSH* must be installed on the host computer and be added to the
   system PATH. SSH is installed with GIT, or you can install it
   independently for your platform.


Set up the Vagrant and the virtual machine
==========================================

#. `Download and install VirtualBox <https://www.virtualbox.org/wiki/Downloads>`__.
#. `Download and install Vagrant <https://www.vagrantup.com/downloads.html>`__ for your
   platform. Windows, OS-X and Linux are supported.
#. Clone the `ArduPilot <https://github.com/ArduPilot/ardupilot>`__
   Github repository anywhere on your PC:

   ::

       git clone https://github.com/ArduPilot/ardupilot.git
       cd ardupilot

#. Start a vagrant instance

   -  Open a command prompt and navigate to any directory in the
      `/ArduPilot/ardupilot/Tools/vagrant/ <https://github.com/ArduPilot/ardupilot/blob/master/Tools/vagrant/>`__
      source tree.
   -  Run the command:

      ::

          vagrant up

    This starts running a VM, based on a *Vagrant configuration file*
    in the source tree. All the files in this directory tree will
    "magically" appear inside the running instance at */vagrant*.

    .. note::

       The first time you run the vagrant up command it will take some
       time complete. The command needs to fetch a Vagrant base VM and
       configure it with the development environment.

#. Initialise git submodules

   - The ArduPilot source tree references other repositories as
     *submodules*.  These must be initialised by working on the
     virtual machine:

      ::

	 vagrant ssh
	 cd /vagrant
	 git submodule update --init --recursive
	 exit

Start running SITL
==================

Enter the following in your vagrant shell to run the Copter simulator.
This will first build the code (if it has not previously been built) and
then run the simulator:

::

    vagrant ssh -c "sim_vehicle.py -j 2 -v ArduCopter"

Once the simulation is running, you will start getting information from
the MAVLink prompt about vehicle state. For example:

::

    GPS lock at 0 meters
    APM: PreArm: RC not calibrated
    APM: Copter V3.3-dev (999710d0)
    APM: Frame: QUAD
    APM: PreArm: RC not calibrated

The Copter Simulator is built in this example, but you can instead build for
the plane or rover by changing the ``-v`` option:

::

    vagrant ssh -c "sim_vehicle.py -j 2 -v ArduPlane"
    vagrant ssh -c "sim_vehicle.py -j 2 -v Rover"

.. tip::

   `sim_vehicle.py <https://github.com/ArduPilot/ardupilot/blob/master/Tools/autotest/sim_vehicle.py>`__
   has many useful build options, ranging from setting the simulation speed
   through to choosing the initial vehicle location. These can be listed by
   calling it with the ``-h`` flag (and some are demonstrated in :ref:`Using SITL for ArduPilot Testing <using-sitl-for-ardupilot-testing>`).

Run Mission Planner or MAVProxy in your main OS
===============================================

You can now connect to the running simulator from your main OS. Just
connect to UDP port 14550, either from *Mission Planner* or *MAVProxy*.
The *MAVProxy* command is:

::

    mavproxy.py --master=127.0.0.1:14550

Shutting down the simulator
===========================

When you are done with the simulator:

-  Press **ctrl-d** in the Vagrant SSH window to exit the special
   *MAVProxy* that is gluing everything together.
-  Suspend the running VM by entering the following in the command
   prompt:

   ::

       vagrant suspend

Restarting the simulator
========================

When you need the simulator again you can resume the VM and restart the
simulator as shown:

::

    vagrant up
    vagrant ssh -c "sim_vehicle.py -j 2"

.. note::

   Restarting the environment usually only takes a few seconds as the
   VM is only suspended and the simulation code for the vehicle has already
   been built.

Updating the simulator
======================

The simulator is built from the source tree shared between the host and
virtual machines, and any changes will trigger a rebuild next time you
start the simulator. To update the simulator you simply need to modify
the source tree (or pull a new version from Github).

Next steps
==========

To get the most out of SITL we recommend you read the :ref:`MAVProxy documentation <mavproxy:home>`.

The topic :ref:`Using SITL for ArduPilot Testing <using-sitl-for-ardupilot-testing>` explains how to use the
simulator, and covers topics like how to use SITL with Ground Stations
other than Mission Planner and MAVProxy.
