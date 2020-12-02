.. _common-autopilots:

==========================
Autopilot Hardware Options
==========================

This section provides information about ArduPilot Autopilot Hardware options.  The list below is sorted by manufacturer and product name.

Open hardware
-------------

.. toctree::
    :maxdepth: 1

    BBBMini* (Linux) <https://github.com/mirkix/BBBMINI>
    Beagle Bone Blue (Linux) <common-beagle-bone-blue>
    CUAV V5 Plus <common-cuav-v5plus-overview>
    CUAV V5 Nano <common-cuav-v5nano-overview>
    CUAV Nora <common-cuav-nora-overview>
    CUAV X7/ X7 Pro <common-cuav-x7-overview>
    Drotek Pixhawk3 <https://drotek.gitbook.io/pixhawk-3-pro/>
    F4BY <common-f4by>
    Hex/ProfiCNC Cube Black <common-thecube-overview>
    Hex/ProfiCNC Cube Orange <common-thecubeorange-overview>
    Hex/ProfiCNC Cube Purple <common-thecubepurple-overview>
    Hex/ProfiCNC Cube Yellow <common-thecubeyellow-overview>
    Hex/ProfiCNC Cube Green <https://docs.cubepilot.org/user-guides/autopilot/the-cube-module-overview>
    Holybro Pix32 v5 <common-holybro-pix32v5>
    mRo Pixhawk <common-pixhawk-overview>
    mRo Pixracer <common-pixracer-overview>
    mRo X2.1 <https://store.mrobotics.io/mRo-X2-1-Rev-2-p/mro-x2.1rv2-mr.htm>
    mRo X2.1-777 <https://store.mrobotics.io/mRo-X2-1-777-p/mro-x2.1-777-mr.htm>
    OpenPilot Revolution <common-openpilot-revo-mini>
    PocketPilot* (Linux) <https://github.com/PocketPilot/PocketPilot>
    TauLabs Sparky2** <common-taulabs-sparky2>

\* these devices are sensor add-on boards for a Beagle Bone microcomputer. See board links for details 

\** due to flash memory limitations, these boards do not include all ArduPilot features. See :ref:`Firmware Limitations <common-limited_firmware>` for details.

Closed hardware
---------------

.. toctree::
    :maxdepth: 1

    Aerotenna Ocpoc-Zynq <https://aerotenna.com/shop/ocpoc-zynq-mini/>
    Emlid NAVIO2 (Linux) <common-navio2-overview>
    Furious FPV F-35 Lightning and Wing FC-10 <common-furiousfpv-f35>
    Holybro Durandal H7 <common-durandal-overview>
    Holybro Kakute F4 <common-holybro-kakutef4>
    Holybro Kakute F7 AIO* <common-holybro-kakutef7aio>
    Holybro Kakute F7 Mini* (only V1 and V2 are compatible) <common-holybro-kakutef7mini>
    Holybro Pixhawk 4 <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/Pixhawk4/README.md>
    Holybro Pixhawk 4 Mini <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/PH4-mini/README.md>
    Mateksys F405-SE <common-matekf405-se>
    Mateksys F405-STD and variants* <common-matekf405>
    Mateksys F405-Wing* <common-matekf405-wing>
    Mateksys F765-Wing <common-matekf765-wing>
    Mateksys H743-Wing <common-matekh743-wing>
    mRo ControlZero F7 <common-mro-control-zero-F7>
    mRo Pixracer Pro (H7) <common-pixracer-pro>
    mRo Nexus <common-mro-nexus>
    Omnibus F4 AIO/Pro* <common-omnibusf4pro>
    OmnibusNanoV6 <common-omnibusnanov6>
    Omnibus F7V2* <common-omnibusf7>
[site wiki="copter"]
    Parrot Bebop Autopilot <parrot-bebop-autopilot>
[/site]
    Parrot C.H.U.C.K <common-CHUCK-overview>
    RadioLink MiniPix <common-radiolink-minipix>
    QioTek Zealot F427 <common-qiotek-zealot>
    SpeedyBee F4 (this board currently is non-verified) <common-speedybeef4>
    VR Brain 5 <http://www.virtualrobotix.it/index.php/en/shop/autopilot/vrbrain5-detail>
    VR uBrain 5.1 <http://www.virtualrobotix.it/index.php/en/shop/autopilot/vrbrainmicro51-detail>

\* due to flash memory limitations, these boards do not include all ArduPilot features. See :ref:`Firmware Limitations <common-limited_firmware>` for details.

.. note:: For more information on using ArduPilot on Linux based boards, see :ref:`building-the-code`

Firmware Limitations
--------------------

Some boards have features removed in order to fit the firmware into their memory capacity. See the section below:

.. toctree::
    :maxdepth: 1

    Firmware Limitations <common-limited-firmware>
    

Discontinued boards
-------------------
The following boards are no longer produced, however documentation is still available in the wiki or online, and recent builds are still expected to work.  These boards are not recommended for new projects.

.. toctree::
    :maxdepth: 1

    CUAV V5 <common-pixhackV5-overview>
    Emlid Edge <common-emlid-edge>
    Erle PXFmini RPi Zero Shield <common-pxfmini>
    Erle ErleBrain <common-erle-brain-linux-autopilot>
    Intel Aero <common-intel-aero-overview>
    Intel Aero RTF vehicle <common-intel-aero-rtf>

The following boards are no longer supported. The documentation is :ref:`archived<common-archived-topics>`, but
available if you're still working on those platforms:

   APM 2.x (APM 2.6 and later) are no longer supported for Copter, Plane or Rover. The last firmware builds that fit on this board are Copter 3.2.1, and Plane 3.4.0, and Rover 2.5.1. 
   NAVIO+ 
   PX4FMU
   Qualcomm Snapdragon Flight Kit

