=======
Modules
=======

MAVProxy can be extended with modules. These run in parallel threads to
the main MAVProxy program in order to maintain overall stability.

Modules can include such things as GUI elements and diagnostic and
monitoring applications.

Module Management
=================

Modules need to be loaded before they can be used. The following command
can be used:

.. code:: bash

    module load modulename

Other management commands for unloading, reloading and listing currently
loaded modules include:

.. code:: bash

    module unload modulename
    module reload modulename
    module list

Default Modules
===============

MAVProxy starts with several modules by default. They are:

.. code:: bash

    adsb: ADS-B data support
    arm: arm/disarm handling
    auxopt: auxopt command handling
    battery: battery commands
    calibration: calibration handling
    cmdlong: cmdlong handling
    fence: geo-fence management
    ftp: ftp handling
    graph: graph control
    layout: window layout handling
    link: link control
    log: log transfer
    misc: misc commands
    mode: mode handling
    msg: 
    output: output control
    param: parameter handling
    rally: rally point control
    rc: rc command handling
    relay: relay handling
    signing: signing control
    terrain: terrain handling
    tuneopt: tuneopt command handling
    wp: waypoint handling


List of Modules
===============

.. toctree::
    :maxdepth: 1
    
    adsb
    antenna
    auxopt
    battery
    cameraview
    cmdlong
    console
    dataflash_logger
    devop
    DGPS
    firmware
    fakegps
    followtest
    ftp
    gasheli
    gimbal
    GPSInput
    graph
    horizon
    joystick
    kmlread
    link
    log
    magical
    map
    misseditor
    movinghome
    park
    ppp
    rcsetup
    relay
    sail
    sensors
    serial
    signing
    speech
    swarm
    systemtime
    terrain
    tracker
    tuneopt
    ublox
    useralerts
    vicon

