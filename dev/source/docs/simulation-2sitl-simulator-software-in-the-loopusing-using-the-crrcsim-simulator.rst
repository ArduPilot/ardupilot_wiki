.. _simulation-2sitl-simulator-software-in-the-loopusing-using-the-crrcsim-simulator:

===========================
Using the CRRCSim simulator
===========================

You can optionally use the CRRCSim flight simulator with ArduPilot SITL.
The main advantages of using CRRCSim is that it offers a lot more fixed
wing models, and also offers a helicopter simulator.

Installing CRRCSim
==================

The CRRCSim simulator has been modified for use by ArduPilot SITL. To
install it on Linux you need to do the following:

Add dependencies

::

    sudo apt-get install plib1.8.4-dev libjpeg-dev libsdl2-dev
    sudo apt-get install libportaudio-dev libcgal-dev #optional for audio/thermal support

To download code and build

::

    git clone git://github.com/tridge/crrcsim-ardupilot.git
    cd crrcsim-ardupilot
    ./autogen.sh
    ./configure
    make
    sudo make install

That will install crrcsim in /usr/local/bin. You may find you are
missing some packages needed for CRRCSim at the configure stage.

Running SITL with CRRCSim
=========================

One crrcsim is installed you can launch it by running:

::

    crrcsim -i APM

that starts CRRCSim with the APM protocol interface. You can then press
ESCAPE to bring up the menu and choose an aircraft to simulate. Many of
the aircraft will work with SITL, but for ones without motors (the
gliders) you will need to choose a launch location on a slope.

For fixed wing testing it is recommended you start with the "Sport"
aircraft. That simulates a small nitro sport aircraft.

For helicopter testing choose the "Heli-APM" model.

After you have launched CRRCSim you need to start SITL. For fixed wing
testing use the "-f CRRCSim" option to sim_vehicle.py:

::

    cd ArduPlane
    sim_vehicle.py -f CRRCSim --console --map

Simulating a helicopter
-----------------------

For helicopter testing with CRRCSim use "-f CRRCSim-heli"

::

    cd ArduCopter
    sim_vehicle.py -f CRRCSim-heli --console --map

The helicopter will have the RSC speed on channel 8.
