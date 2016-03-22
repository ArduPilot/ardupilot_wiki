.. _droneapi-tutorial:

=================
DroneKit Tutorial
=================

This tutorial is a basic introduction to using the python version of the
DroneAPI.

.. note::

   This tutorial superseded (and is part of) the `official DroneKit Python documentation <http://python.dronekit.io/>`__.

This is a alpha-release for developer feedback, so please post any
questions or problems encountered to our github `issue tracker <https://github.com/dronekit/dronekit-python/issues>`__, if
github is not suitable for some reason please send email to
kevin@3drobotics.com. I promise to quickly respond with fixes.

Installing
==========

The DroneAPI is available in the public pypi repository.  Therefore on
essentially any machine that can run python you can use the pip tool to
install.

Linux:
------

If you are Ubuntu you can get pip (and other required dependencies) by
running:

::

    sudo apt-get install pip python-numpy python-opencv python-serial python-pyparsing python-wxgtk2.8

::

    sudo pip install droneapi

OS X:
-----

Install WXMac

::

    brew install wxmac

Install the following python libraries

::

    pip install numpy pyparsing

On OSX you need to uninstall python-dateutil since osx comes bundled
with a version that is not supported for some dependencies

::

    pip uninstall python-dateutil

Finally install the droneapi:

::

    pip install droneapi

Windows:
--------

The windows installation is a little more involved, but not too hard.

You could install the various python libraries by hand, but we think
that it is easier to use the WinPython package. The steps to install
this package and our add-on modules are:

**1.** Run the correct `WinPython installer for your platform <http://sourceforge.net/projects/winpython/files/WinPython_2.7/2.7.6.4/>`__
(win32 vs win64)

**2.** Register the python that came from **WinPython** as the preferred
interpreter for your machine:

Open the folder where you installed WinPython, run "**WinPython Control
Panel**\ " and choose "**Advanced/Register Distribution**\ ".

.. image:: ../../../images/dronekit_winpython_command_prompt.png
    :target: ../_images/dronekit_winpython_command_prompt.png

Screenshot of this step (click for larger version)

**3.** Run "**WinPython Command Prompt**\ " and run the following two
commands:

::

    pip uninstall python-dateutil
    pip install droneapi

**4.** Done!, You can now run "**mavproxy.py --master=COM3**\ " (etc...)
as needed in the tutorial steps below...

Hello world: a first real world example
=======================================

A few example applications are included with the droneapi-python
package.

Installing the example code
---------------------------

For this tutorial you'll probably want the example files contained
within the package source, to get those examples

::

    git clone http://github.com/dronekit/dronekit-python.git

Let's start by just running one of those, the first step is to change
your current directory:

::

    cd droneapi-python/examples

Starting MAVProxy
-----------------

When developing new DroneAPI python code the easiest approach is to run
it inside of MAVProxy. So launch MAVProxy with the correct options for
talking to your vehicle:

**Linux**:

::

    mavproxy.py --master=/dev/ttyUSB0

**OSX**:

::

    mavproxy.py --master=/dev/cu.usbmodem1

**Windows**:

::

    mavproxy.py --master=com3:

**For other connection options see the MAVProxy
`documentation <http://dronecode.github.io/MAVProxy/html/getting_started/starting.html>`__.**

Loading the API
---------------

The API includes a mavproxy
`module <http://dronecode.github.io/MAVProxy/html/modules/index.html>`__ to
allow you to load (and reload) your custom application into mavproxy.

To load the API module run:

::

    MANUAL> module load droneapi.module.api
    DroneAPI loaded
    MANUAL>

We recommend adding this line to the mavproxy `startup script in **~/.mavinit.scr** <http://dronecode.github.io/MAVProxy/html/getting_started/mavinit.html>`__.

::

    echo "module load droneapi.module.api" >> ~/.mavinit.scr

Running the example
-------------------

The first example we will run is a very small application that just
reads some vehicle state and then changes the vehicle mode to AUTO (to
start following prestored waypoints).

.. warning::

   For all of these examples, please run them initially with a
   vehicle at your desk with props removed.

**It is probably best to take a look at `the python code <https://github.com/dronekit/dronekit-python/blob/master/examples/vehicle_state/vehicle_state.py>`__
before running it.**

::

    MANUAL> api start small_demo.py
    Mode: VehicleMode:MANUAL
    Location: Location:lat=21.2938874,lon=-157.8501416,alt=0.189999997616,is_relative=None
    Attitude: Attitude:-0.286077767611,-3.01412272453,0.261489063501
    GPS: GPSInfo:fix=1,num_sat=0
    ...

Follow-Me
=========

This is a significantly more complex example - showing closed-loop
control of the vehicle. It will use a USB GPS attached to your laptop to
have the vehicle follow you as you walk around a field.

.. warning::

   Run this example with caution - be ready to exit follow-me mode
   by switching the flight mode switch on your RC radio, this is especially
   true because there is currently a `bug in the APM code which makes
   follow-me very 'twitchy/unstable' for some
   configurations <https://github.com/diydrones/ardupilot/issues/879>`__.

In practice, you don't really want to use this follow-me implementation,
rather you can use this example as a starting point to build your own
custom application.

Before running this demo you'll need to make sure your computer has the
`gpsd <http://www.catb.org/gpsd/>`__ service installed.

**Ubuntu**:

::

     apt-get install gpsd gpsd-clients

You can then plug in a USB GPS and run the **"xgps"** client to confirm
that it is working. If you do not have a USB GPS you can use simulated
data by running **droneapi-python/examples/run-fake-gps.sh**.

Once your GPS is plugged in you can start follow-me by running the
following command inside of MAVProxy:

::

    RTL> api start follow_me.py
    RTL> Going to: Location:lat=50.616468333,lon=7.131903333,alt=30,is_relative=True
    Got MAVLink msg: MISSION_ACK {target_system : 255, target_component : 0, type : 0}
    GUIDED> Mode GUIDED
    Going to: Location:lat=50.616468333,lon=7.131903333,alt=30,is_relative=True
    Got MAVLink msg: MISSION_ACK {target_system : 255, target_component : 0, type : 0}
    ...

These debugging messages will appear every two seconds - when a new
target position is sent to the vehicle, to stop follow-me either change
the vehicle mode switch on your RC transmitter or type "**api stop**\ ".

The `source code for this example <https://github.com/dronekit/dronekit-python/blob/master/examples/follow_me/follow_me.py>`__
is a good starting point for your own application, from here you can use
all python language features and libraries (OpenCV, classes, lots of
packages etc...)

Next steps
==========

Good next steps is to read the DroneAPI developers guide.

Contact:
========

Kevin Hester kevin@3drobotics.com

Issue tracking: https://github.com/dronekit/dronekit-python/issues
