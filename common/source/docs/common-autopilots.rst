.. _common-autopilots:

=====================
Choosing an Autopilot
=====================

ArduPilot runs on many different autopilot boards. Selecting the right board depends on the physical constraints of the
vehicle, features desired, and the applications that you want to run.
Factors to consider are:

#. Sensor Redundancy: ArduPilot supports redundant IMUS, GPS, etc. Many controllers have multiple IMUs integrated on board for applications requiring this level of reliability.
#. Number of Servo/Motor Outputs.
#. Number of UARTs: Telemetry radios, GPS's, Companion Computers, etc can be attached via these ports.
#. External Buses: I2C, CAN, etc. allow many types of devices, such as airspeed sensors, LED controllers, etc. to be attached to the autopilot.
#. Number of Analog I/O: Some controllers have analog I/O available for such features as inputting receiver signal strength (RSSI) or battery voltage/current or other analog sensors.
#. Integrated Features: Such as on-board OSD (On Screen Display), integrated battery monitoring sensors.
#. Vibration Isolation: Internal mechanical vibration isolators for IMUs for high vibration applications.
#. IMU Heaters: On board temperature control of IMUs for applications in harsh environments or widely varying temperatures during a flight to provide the highest possible precision.
#. Size: Many vehicles have limited space for the autopilot.
#. Expense: Controller prices range from ~$25 to much more, depending on feature set.

The sections below provide information about ArduPilot autopilot hardware options.  The list below is sorted by manufacturer and product name.

.. tip::

   There are also numerous clones and minor variants of the boards linked below. Many of these may be perfectly capable replacements. Help in selecting an autopilot can also be obtained in the user forums (`ArduPilot Discuss <https://discuss.ardupilot.org>`__ or `RC Groups <https://www.rcgroups.com/forums/showthread.php?3102183-Most-Integrated-Ardupilot-Flight-Controller-%28ChiBios%29>`__ ).


[site wiki="copter, plane" heading="off"]

.. note::

   The APM2.6 board is no longer supported for Copter or Plane. The
   last firmware builds that can be installed on these boards are AC v3.2.1 and Plane
   3.3.0.

[/site]

.. note:: Not all of these autopilots have been directly tested by ArduPilot development team members and while firmware  is provided by ArduPilot, it does not constitute an endorsement by ArduPilot.  However, most boards developed by ArduPilot Partners have been provided to the ArduPilot development team to aid in any possible support issues that might arise. 


Open Hardware
=============

.. toctree::
    :maxdepth: 1

    ARKV6X DS-10 Pixhawk6 <https://arkelectron.com/product/arkv6x/>
    CUAV V5 Plus <common-cuav-v5plus-overview>
    CUAV V5 Nano <common-cuav-v5nano-overview>
    CUAV Nora <common-cuav-nora-overview>
    CUAV Pixhawk v6X <common-cuav-pixhawkv6X>
    CUAV X7/X7Pro/X7+/X7+ Pro <common-cuav-x7-family-overview>
    Drotek Pixhawk3 <https://drotek.gitbook.io/pixhawk-3-pro/>
    F4BY <common-f4by>
    CubePilot Cube Black <common-thecube-overview>
    CubePilot Cube Orange/+ <common-thecubeorange-overview>
    CubePilot Cube Purple <common-thecubepurple-overview>
    CubePilot Cube Yellow <common-thecubeyellow-overview>
    CubePilot Cube Green <https://docs.cubepilot.org/user-guides/autopilot/the-cube-module-overview>
    Holybro Durandal H7 <common-durandal-overview>
    Holybro Pix32 v5 <common-holybro-pix32v5>
    Holybro Pixhawk 4 <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/Pixhawk4/README.md>
    Holybro Pixhawk6X <common-holybro-pixhawk6X>
    Holybro Pixhawk6C/ 6C Mini <common-holybro-pixhawk6C>
    Holybro Pix32v6(Pixhawk6C variant) <common-holybro-pix32v6>
    mRo Pixhawk <common-pixhawk-overview>
    mRo Pixracer <common-pixracer-overview>
    mRo X2.1 <https://store.mrobotics.io/mRo-X2-1-Rev-2-p/m10021a.htm>
    mRo X2.1-777 <https://store.mrobotics.io/mRo-X2-1-777-p/m10022a.htm>
    OpenPilot Revolution <common-openpilot-revo-mini>
    TauLabs Sparky2 <common-taulabs-sparky2>

\* due to flash memory limitations, these boards do not include all ArduPilot features. See :ref:`Firmware Limitations <common-limited_firmware>` for details.

Closed Hardware
===============

.. toctree::
    :maxdepth: 1

    Aerotenna Ocpoc-Zynq <https://aerotenna.com/shop/ocpoc-zynq-mini/>
    Airvolute DroneCore <common-airvolute-DroneCore-Suite>
    AnyleafH7 <common-anyleafh7>
    AocodaRC H743Dual <common-aocoda-h743dual>
    AtomRC F405-NAVI <common-atomrcf405-navi>
    BetaFPV F4 1S 12A AIO V3 <common-betafpvf405>
    brainFPV RADIX2 HD <common-radix2hd>
    Emlid NAVIO2 (Linux) <common-navio2-overview>
    Flywoo F405 Pro <common-flywoof405pro>
    Flywoo F745 AIO BL_32/ Nano <common-flywoo-f745>
    Foxeer H743 MPU600 <common-foxeerh743v1>
    Foxeer Reaper F745-AIO V2 <common-foxeerf745aio>
    Furious FPV F-35 Lightning and Wing FC-10 <common-furiousfpv-f35>
    HeeWing F405/F405V2 <common-heewingf405>
    Holybro Kakute F4 <common-holybro-kakutef4>
    Holybro Kakute F4 Mini <common-holybro-kakutef4-mini>
    Holybro Kakute F7 AIO <common-holybro-kakutef7aio>
    Holybro Kakute F7 Mini (only V1 and V2 are compatible) <common-holybro-kakutef7mini>
    Holybro Kakute H7 V1 <common-holybro-kakuteh7.rst>
    Holybro Kakute H7 V2 <common-holybro-kakuteh7-v2>
    Holybro Kakute H7 Mini v1.3 <common-holybro-kakuteh7mini-v13>
    Holybro Kakute H7 Wing <common-kakuteh7wing>
    Holybro Pixhawk 4 Mini <common-holybro-ph4mini>
    Holybro Pixhawk5X <common-holybro-ph5x>
    Horizon31 PixC4-Jetson <common-horizon31-pixc4-jetson>
    IFlight Beast F7 45A AIO <common-iflight-beastf7AIO>
    IFlight BeastH7 AIO <common-iflight-beasth7AIO>
    IFlight Blitz Whoop F7 AIO <common-iflight-blitzf7AIO>
    JAE JFB-110 <common-jae-jfb110>
    Mamba F405 MK2 <common-mamba405-mk2>
    Mamba MK4 F405Mini <common-mambaf405-mini>
    Mamba Basic F405 mk3 <common-mamba-basic-mk3>
    Mamba H743 v4 <common-mambaH743v4>
    MakeFlyEasy PixSurveyA1 <common-makeflyeasy-PixSurveyA1>
    MakeFlyEasy PixSurveyA1-IND <common-makeflyeasy-PixSurveyA1-IND>
    MakeFlyEasy PixSurveyA2 <common-makeflyeasy-PixSurveyA2>
    MakeFlyEasy PixPilot-C3 <common-makeflyeasy-PixPilot-C3>
    MakeFlyEasy PixPilot-V3 <common-makeflyeasy-PixPilot-V3>
    MakeFlyEasy PixPilot-V6 <common-makeflyeasy-PixPilot-V6>
    Mateksys F405 TE Family <common-matekf405-te>
    Mateksys H743-Wing/MINI/SLIM/WLITE <common-matekh743-wing>
    ModalAI Flight core <https://www.modalai.com/products/flight-core>
    mRo ControlZero Classic <https://store.mrobotics.io/product-p/m10048d.htm>
    mRo ControlZero F7 <common-mro-control-zero-F7>
    mRo ControlZero H7 <https://store.mrobotics.io/mRo-Control-Zero-H7-p/mro-ctrl-zero-h7.htm>
    mRo COntrolZero H7 OEM <https://store.mrobotics.io/product-p/m10059d.htm>
    mRo Pixracer Pro (H7) <common-pixracer-pro>
    mRo Nexus <common-mro-nexus>
    Omnibus F4 AIO/Pro <common-omnibusf4pro>
    OmnibusNanoV6 <common-omnibusnanov6>
    Omnibus F7V2 <common-omnibusf7>
[site wiki="copter"]
    Parrot Bebop Autopilot <parrot-bebop-autopilot>
[/site]
    Parrot C.H.U.C.K <common-CHUCK-overview>
    RadioLink MiniPix <common-radiolink-minipix>
    QioTek Zealot F427 <common-qiotek-zealot>
    QioTek Zealot H743 <common-qiotek-zealoth7>
    SDMODEL SDH7V! <https://www.sdmodel.com.tw/content.php?cn=item&tid=1320&start=0>
    Sky-Drones AIRLink <common-skydrones-airlink>
    SkystarsH7HD <common-skystarsH7>
    SPRacing H7 Extreme <common-spracingh7-extreme>
    SPRacing H7 RF <common-spracingh7-rf>
    Swan-K1 <common-Swan-K1>
    SpeedyBee F4 (this board currently is non-verified) <common-speedybeef4>
    SpeedyBee F4 V3/V4 <common-speedybeef4-v3>
    SpeedyBee F405 Mini <common-speedybeef405-mini>
    SpeedyBeeF405WING/WING Mini <common-speedybeef405wing>
    ThePeach FCC-K1 <common-thepeach-k1>
    ThePeach FCC-R1 <common-thepeach-r1>
    VR Brain 5 <http://www.virtualrobotix.it/index.php/en/shop/autopilot/vrbrain5-detail>
    VR uBrain 5.1 <http://www.virtualrobotix.it/index.php/en/shop/autopilot/vrbrainmicro51-detail>
    YJUAV A6SE <common-yjuav-a6se>
    YJUAV A6SE H743<common-yjuav-a6se-h743>

.. note:: due to flash memory limitations, most F4 based, and some other boards, do not include all ArduPilot features. See :ref:`Firmware Limitations <common-limited_firmware>` for details.

Linux Based Autopilots
======================

These autopilots use an underlying Linux OS. Linux boards usually have more CPU power and memory many of the other boards listed on this page, but do not support DShot, Bi-Directional DShot, BLHeli ESC passthrough, many of the ArduPilot GPIO based features, and easy upload from the ground stations. They do allow experimentation and development of advanced control and navigation algorithms (see also :ref:`common-companion-computers`).

.. toctree::
    :maxdepth: 1

    BBBMini** (Linux) <https://github.com/mirkix/BBBMINI>
    Beagle Bone Blue (Linux) <common-beagle-bone-blue>
    Blue Robotics Navigator** (Linux) <https://bluerobotics.com/store/comm-control-power/control/navigator/>
    Obal Board (Linux) <common-obal-overview>
    PocketPilot** (Linux) <https://github.com/PocketPilot/PocketPilot>

\** these devices are sensor add-on boards for a Linux-based microcomputer. See board links for details.

.. note:: For more information on using ArduPilot on Linux based boards, see :ref:`building-the-code`

Firmware Limitations
====================

Some boards have features removed in order to fit the firmware into their memory capacity. See the section below:

.. toctree::
    :maxdepth: 1

    Firmware Limitations <common-limited-firmware>
    
.. note:: If a board has a missing feature that is required by the user, building a custom firmware using the `ArduPilot Custom Firmware Build Server <https://custom.ardupilot.org>`__ can be used to create firmware which drops features not needed and adding desired features back into the firmware in that freed space.

Discontinued Boards
===================
The following boards are no longer produced, however, documentation is still available in the wiki or online, and recent builds are still expected to work.  These boards are not recommended for new projects.

.. toctree::
    :maxdepth: 1

    CUAV V5 <common-pixhackV5-overview>
    Emlid Edge <common-emlid-edge>
    Erle PXFmini RPi Zero Shield <common-pxfmini>
    Erle ErleBrain <common-erle-brain-linux-autopilot>
    Holybro Kakute H7 Mini <common-holybro-kakuteh7mini.rst>
    Intel Aero <common-intel-aero-overview>
    Intel Aero RTF vehicle <common-intel-aero-rtf>
    Mateksys F405-SE <common-matekf405-se>
    Mateksys F405-STD and variants* <common-matekf405>
    Mateksys F405-Wing <common-matekf405-wing>
    Mateksys F765-Wing <common-matekf765-wing>
    Mateksys F765-WSE <common-matekf765-wse>

The following boards are no longer supported. The documentation is :ref:`archived<common-archived-topics>`, but
available if you're still working on those platforms:

- APM 2.x (APM 2.6 and later) are no longer supported for Copter, Plane or Rover. The last firmware builds that fit on this board are Copter 3.2.1, and Plane 3.4.0, and Rover 2.5.1. 
- NAVIO+ 
- PX4FMU
- Qualcomm Snapdragon Flight Kit

Schematics
==========

Schematics for some of the "Open Hardware" autopilots `can be found here <https://github.com/ArduPilot/Schematics>`__


[copywiki destination="plane,copter,rover,blimp"]
