.. _security-landing-page:

========
Security
========

This page describes how to protect an ArduPilot vehicle from external threats.

.. toctree::
    :maxdepth: 1

    MAVLink2 Signing <common-MAVLink2-signing>
    Parameter Lockdown <common-parameter-lockdown>
    Secure Firmware (tamper-proof) <secure-firmware>
    Remote ID <common-remoteid>

The first step in protecting the vehicle is to ensure that it is physically secure.  If a malicious individual has physical access to the vehicle there are numerous ways they could endanger the system including cutting wires, breaking propellers, adding devices that interfere with the sensors (vibration, magnetic interference, EMI).  While the ArduPilot software has `pre-arm checks <https://ardupilot.org/copter/docs/common-prearm-safety-checks.html>`__ that may catch some methods of vandalism, it cannot protect against them all.

While security is important, some security measures can affect usability so a balanced approach keeping in mind the user's needs and rights under the :ref:`GPLv3 license <license-gplv3>` is also important.

Security Attack Surface
-----------------------

The "security attack surface" is the set of access points that a malicious actor could use to gain access to the autopilot and disrupt its operation.  Below is a table of these surfaces with advice on how they may be protected.  These surfaces are all considered trusted by default.

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Access Point
     - Recommendations
   * - CAN
     - Used for many `peripherals <https://ardupilot.org/copter/docs/common-uavcan-peripherals.html>`__ including GPSs, Lidar, ESCs
   * - `Ethernet <https://ardupilot.org/copter/docs/common-network.html>`__
     - Used primarily for :ref:`companion computers <companion-computers>` and `camera gimbals <https://ardupilot.org/copter/docs/common-cameras-and-gimbals.html>`__ and protection should be applied to their attack surfaces
   * - GPS
     - Susceptible to jamming and spoofing.  The `EKF failsafe <https://ardupilot.org/copter/docs/ekf-inav-failsafe.html>`__ may trigger a failsafe action but consider adding `Non-GPS navigation <https://ardupilot.org/copter/docs/common-non-gps-navigation-landing-page.html>`__ as a backup
   * - I2C / SPI
     - Used for IMU and some `peripherals <https://ardupilot.org/copter/docs/common-optional-hardware.html>`__ including compass and lidar
   * - RC Receiver Input
     - Used for pilot control of the vehicle in manual `flight modes <https://ardupilot.org/copter/docs/flight-modes.html>`__, `flight mode selection <https://ardupilot.org/copter/docs/common-rc-transmitter-flight-mode-configuration.html>`__ and `auxiliary switch functions <https://ardupilot.org/copter/docs/common-auxiliary-functions.html>`__.  Check the attack surface of the pilot's transmitter and the RC receiver on the vehicle.  Use the ``RC_PROTOCOLS`` parameter to enable only the protocols being used.
   * - SD Card
     - Accessed physically or via :ref:`MAVFtp <mavlink-mavftp>`.  Used for `log files <https://ardupilot.org/copter/docs/common-logs.html>`__, `rally points <https://ardupilot.org/copter/docs/common-rally-points.html>`__, `terrain data storage <https://ardupilot.org/plane/docs/common-terrain-following.html>`__, `Lua scripts <https://ardupilot.org/copter/docs/common-lua-scripts.html>`__ and (optionally) `firmware installation <https://ardupilot.org/copter/docs/common-install-sdcard.html>`__.  Physically secure the SD card, enable :ref:`MAVLink2 Signing <common-MAVLink2-signing>` and check the attack surfaces of the telemetry connections.  Include :ref:`Lua scripts directly in the firmware <common-oem-customizations>`
   * - Serial Ports
     - Used for MAVLink and `peripherals <https://ardupilot.org/copter/docs/common-optional-hardware.html>`__.  Check the attack surface of the peripheral and/or enable :ref:`MAVLink2 Signing <common-MAVLink2-signing>`.  See "Ground Control Stations" section below
   * - USB port
     - Used for both firmware uploads and MAVLink communication.  See :ref:`Secure Firmware <secure-firmware>` for instructions on installing a tamper-proof bootloader.  :ref:`MAVLink2 Signing <common-MAVLink2-signing>` is not supported but setting SERIAL0_PROTOCOL = 0 disables MAVLink on this port

Please note that ArduPilot is very often operating in a resource-constrained environment meaning it cannot, on its own, provide all the checks required to ensure a bulletproof system.  Instead the system as a whole must be designed to be secure.  To this end, by default, the inputs to the autopilot (aka "attack surfaces") are trusted.  We trust our SPI-connected inertial sensors to be well-behaved, for example.

Ground Control Stations
.......................

Most notable amongst "trusted" data sources are connections to the Ground Control Station.  We expect GCSs to be well-behaved in terms of the data sent to the autopilot.  Remember that your GCS can disarm your vehicle mid-air or force it into terrain as a matter of course.

We disable floating point exceptions in the embedded firmware, meaning that a lot of floating point operations which would result in a Floating Point Exception now simply don't.  By default we do NOT disable floating point exceptions in SITL, allowing errors in Ground Control Stations to be picked up in SITL rather than when someone is flying a real vehicle.  To more-closely approximate what happens on our embedded platforms, you can use the ``SIM_FLOAT_EXCEPT`` parameter to disable floating point exceptions in ArduPilot SITL.  This may help find real problems when trying to use fuzzers to find problems with the ArduPilot codebase.

One exception to the trusted-ground-control-station model is if :ref:`MAVLink2 Signing <common-MAVLink2-signing>` is enabled.  If the serial port is configured for MAVLink2 signing, unsigned packets will be ignored and should not cause the vehicle to misbehave with the possible exception of DOS attacks.  If you believe you have found a situation where an unsigned packet on a MAVLink2 signed connection causes unexpected behaviour please see the "Reporting Security Issues" section below.

Parameter Lockdown
------------------

Parameters may be locked-down to prevent updates using either of these methods:

- :ref:`Parameter Lockdown Lua script <common-parameter-lockdown>`
- Add the parameter to the defaults.parm file with ``@READONLY`` next to it as described in :ref:`OEM Customizations <common-oem-customizations>`

Feature Reduction
-----------------

The `custom build server <https://ardupilot.org/copter/docs/common-custom-firmware.html>`__ (`direct link <https://custom.ardupilot.org/>`__) may be used to create a firmware with only the minimum set of features and drivers required for the vehicle.  This reduces the amount of code running on the autopilot and thus the potential attack surface.

Disable Driver Auto Detection
-----------------------------

Some drivers auto-detect the sensor being used.  Similar to feature reduction, disabling auto-detection can reduce the attack surface by limiting the code that interacts with the hardware.  Examples include:

- ``BARO_PROBE_EXT``
- ``COMPASS_DISBLMSK``
- ``GPS1_TYPE``, ``GPS2_TYPE``
- ``NTF_LED_TYPES``
- ``RC_PROTOCOLS``

Pre-arm Checks Customisation
----------------------------

Custom pre-arm checks can be added using Lua scripts including :ref:`building the scripts directly into the firmware <common-oem-customizations>`.  Below are some examples that may be used as-is or as a starting point for your own custom checks:

- `applets/arming-checks.lua <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/applets/arming-checks.lua>`__
- `examples/arming-check-batt-temp.lua <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/examples/arming-check-batt-temp.lua>`__
- `examples/arming-check-wp1-takeoff.lua <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/examples/arming-check-wp1-takeoff.lua>`__

Please refer to the `Scripts Crash and Errors <https://ardupilot.org/copter/docs/common-lua-scripts.html#script-crashes-and-errors>`__ wiki section for information on how to ensure scripts are run reliably and not modified.

Build the Code Yourself
-----------------------

The `ArduPilot firmware server <https://firmware.ardupilot.org/>`__ provides pre-compiled firmware for a wide range of boards and configurations.  The core dev team diligently maintains the server including applying security patches and updates.  However, if you want to be sure that the code running on your autopilot is exactly the code you have reviewed, consider :ref:`building the code yourself <building-the-code>`.

Reporting Security Issues
-------------------------

If you believe you have found a security issue in ArduPilot, please raise an issue in the `issues list <https://github.com/ArduPilot/ardupilot/issues>`__ and optionally post a link to the issue in the :ref:`ArduPilot Discord Server's <ardupilot-discord-server>` "code_review" channel.

While we are keen to hear of potential security issues, before reporting, please read the "Security Attack Surface" and "Ground Control Station" sections above.  The most common reports we receive are from `fuzz testing <https://en.wikipedia.org/wiki/Fuzzing>`__ involving changing various parameters in SITL and causing the vehicle to crash or sending in invalid MAVLink messages.  These are generally not valid security issues because the ground station is considered trusted. 

Common Misconceptions
---------------------

*"ArduPilot is open source, so it must be insecure"*

Absolutely incorrect.  The "open" in "open source" refers to the fact that the source code is available for anyone to read which contributes to security by having more sympathetic eyes finding and reporting issues.  This does not mean that anyone can change it, nor does it mean that it must have "open" attack surfaces.  ArduPilot's open source software is "compiled" before use (as is all C++ software), once compiled there's no fundamental difference between open and closed source software.  Here are some supporting articles:

  - `efrontl's Open-source and the “security through obscurity” fallacy <https://www.efrontlearning.com/blog/2012/04/open-source-and-the-security-through-obscurity-fallacy.html>`__
  - `PingCap's Evaluating Security: Open Source vs Proprietary Software <https://www.pingcap.com/article/evaluating-security-open-source-vs-proprietary-software/>`__
  - `LinuxFoundation's Combating Cyber Threats: The Power of Open Source <https://www.linuxfoundation.org/blog/combating-cyber-threats-the-power-of-open-source/>`__
  - `SimpleRisk's The Security of Open Source vs Closed Source Software <https://www.simplerisk.com/blog/security-open-source-vs-closed-source-software>`__
  - `WilsonCenter's Open Source Software and Cybersecurity: How unique is this problem? <https://www.wilsoncenter.org/blog-post/open-source-software-and-cybersecurity-how-unique-problem>`__
  - `Pingdom's 75 Percent of Top 10K Websites Served by Open Source Software <https://www.pingdom.com/blog/75-percent-top-10k-websites-served-by-open-source-software/>`__ (shows how open source can be safe)

*"Open source software is lower quality than closed source software"*

Completely false.  Code quality is determined by the developers, the review process, the amount of testing and release procedures, not by whether it is open or closed source.

The core ArduPilot development team is very focused on code quality and has a rigorous code review process.  Each change (aka Pull Request) must pass hundreds of automated checks and be reviewed by one or more dev team members before being "merged" into the main code base.  After merging the combined code is continuously subject to automated testing and real flight testing by the core dev team and community members before even reaching the beta testing stage.  Beta testing of major releases normally takes 3 months during which time partner companies and community members test the software on a wide range of vehicles.  Incident reports are logged, investigated and fixes applied as required.  Only after beta testing is complete is the software released as the stable/official version.  Post release, the core dev team carefully monitors the forums for incidents and prioritises bug fixes over enhancements, dutifully reporting critical issues in a timely manner to the community.
