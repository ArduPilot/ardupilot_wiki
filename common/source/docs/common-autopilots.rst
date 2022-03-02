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
    CubePilot Cube Black <common-thecube-overview>
    CubePilot Cube Orange <common-thecubeorange-overview>
    CubePilot Cube Purple <common-thecubepurple-overview>
    CubePilot Cube Yellow <common-thecubeyellow-overview>
    CubePilot Cube Green <https://docs.cubepilot.org/user-guides/autopilot/the-cube-module-overview>
    Holybro Durandal H7 <common-durandal-overview>
    Holybro Pix32 v5 <common-holybro-pix32v5>
    Holybro Pixhawk 4 <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/Pixhawk4/README.md>
    mRo Pixhawk <common-pixhawk-overview>
    mRo Pixracer <common-pixracer-overview>
    mRo X2.1 <https://store.mrobotics.io/mRo-X2-1-Rev-2-p/m10021a.htm>
    mRo X2.1-777 <https://store.mrobotics.io/mRo-X2-1-777-p/m10022a.htm>
    Obal Board (Linux) <common-obal-overview>
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
    Flywoo F745 AIO BL_32 <common-flywoo-f745>
    Holybro Kakute F4* <common-holybro-kakutef4>
    Holybro Kakute F4 Mini* <common-holybro-kakutef4-mini>
    Holybro Kakute F7 AIO* <common-holybro-kakutef7aio>
    Holybro Kakute F7 Mini* (only V1 and V2 are compatible) <common-holybro-kakutef7mini>
    Holybro Kakute H7-BT <common-holybro-kakuteh7.rst>
    Holybro Pixhawk 4 Mini <common-holybro-ph4mini>
    Holybro Pixhawk5X <common-holybro-ph5x>
    Horizon31 PixC4-Jetson <common-horizon31-pixc4-jetson>
    IFlight Beast F7 45A AIO <common-iflight-beastf7AIO>
    IFlight BeastH7 AIO <common-iflight-beasth7AIO>
    Mamba F405 MK2* <common-mamba405-mk2>
    Mamba Basic F405 mk3 <common-mamba-basic-mk3>
    Mateksys F405-SE <common-matekf405-se>
    Mateksys F405-STD and variants* <common-matekf405>
    Mateksys F405-Wing* <common-matekf405-wing>
    Mateksys F765-Wing <common-matekf765-wing>
    Mateksys F765-WSE <common-matekf765-wse>
    Mateksys H743-Wing/MINI/SLIM <common-matekh743-wing>
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
    QioTek Zealot H743 <common-qiotek-zealoth7>
    Sky-Drones AIRLink <common-skydrones-airlink>
    Swan-K1 <common-Swan-K1>
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

Schematics
----------

Schematics for some of the "Open hardware" autopilots `can be found here <https://github.com/ArduPilot/Schematics>`__
