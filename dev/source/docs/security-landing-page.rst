.. _security-landing-page:

========
Security
========

This pages provides an overview of security considerations for ArduPilot users and developers, with specific sections for major software components of the ArduPilot ecosystem.


Vehicle Considerations
======================

From a user perspective, the following resources are available to help secure your vehicle:

.. toctree::
    :maxdepth: 1

    MAVLink2 Signing <common-MAVLink2-signing>
    Parameter Lockdown <common-parameter-lockdown>
    Secure Firmware (tamper-proof) <secure-firmware>
    Remote ID <common-remoteid>

The first step in protecting the vehicle is to ensure that it is physically secure.  If a malicious individual has physical access to the vehicle there are numerous ways they could endanger the system including cutting wires, breaking propellers, adding devices that interfere with the sensors (vibration, magnetic interference, EMI).  While the ArduPilot software has `pre-arm checks <https://ardupilot.org/copter/docs/common-prearm-safety-checks.html>`__ that may catch some methods of vandalism, it cannot protect against them all.

While security is important, some security measures can affect usability so a balanced approach keeping in mind the user's needs and rights under the :ref:`GPLv3 license <license-gplv3>` is also important.

ArduPilot Firmware
==================

The ArduPilot security boundary extends up to, but does not include, the physical communication interface (radio link, network cable, USB cable, etc.).  Security of the Command and Control (C2) link and any vehicle-based network is the responsibility of the user.

Users must understand which ports and interfaces are enabled on their autopilot and what data each one processes.  Disable any interface that is not required for the intended use case.  See the "Security Attack Surface" section below for a breakdown of each interface and recommended restrictions.

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
     - Used primarily for MAVLink and `peripherals <https://ardupilot.org/copter/docs/common-optional-hardware.html>`__ and protection should be applied to their attack surfaces.  Any vehicle-based LAN must be kept secure — treat it as a trusted but isolated network and prevent unauthorised physical or wireless access.
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

A key design goal of ArduPilot is that receiving corrupted or malformed data on any interface must not halt or crash the software.  Robustness against bad data is treated as a correctness requirement, not merely a security nicety.  If you observe ArduPilot crashing or hanging upon receipt of malformed data, please report it as a bug via the process described in the "Reporting Security Issues" section below.

Some ArduPilot configurations support an IP server (ie HTTP) running directly on the autopilot (for example via :ref:`AP_Networking <common-network>`).  Any such interface will advise the user that a server is active and display the address and port on which it is listening.  Users are responsible for ensuring that vehicle-side IP interfaces are accessible only to trusted network participants.

We disable floating point exceptions in the ArduPilot firmware, meaning that a lot of floating point operations which would result in a Floating Point Exception now simply don't.  By default we do NOT disable floating point exceptions in SITL, allowing errors in Ground Control Stations to be picked up in SITL rather than when someone is flying a real vehicle.  To more-closely approximate what happens on our embedded platforms, you can use the ``SIM_FLOAT_EXCEPT`` parameter to disable floating point exceptions in ArduPilot SITL.  This may help find real problems when trying to use fuzzers to find problems with the ArduPilot codebase.

Lua Scripts
-----------

:ref:`LUA Scripts <common-lua-scripts>` are a powerful way to add custom functionality to ArduPilot.  However, they can also be a security risk if they are modified by an malicious actor or if they contain vulnerabilities that can be exploited.  To mitigate these risks, consider the following recommendations:

- Only use Lua scripts from trusted sources (such as the ArduPilot Github repository) and review the code of any scripts before use

Lua scripts from the ArduPilot Github repository are reviewed by the ArduPilot Dev Team and are generally considered safe to use.  However, if you find a security issue in one of these scripts, please report it via the "Reporting Security Issues" process described below. Each script is documented (where applicable) with any known security considerations - such as unauthenticated interfaces - and any such considerations should be reviewed before use.

Note that :ref:`AP_Periph <dev:ap-peripheral-landing-page>` nodes can also run Lua scripts and the same recommendations apply to them as well.

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

Ground Control Stations
=======================

We expect GCSs to be well-behaved in terms of the data sent to the autopilot. Remember that your GCS can disarm your vehicle mid-air or force it into terrain as a matter of course.

All ArduPilot GCS's will clearly inform the user which ports and network addresses they are auto-connecting to or auto-listening on at startup.  Users should verify this information and ensure that only expected connections are active.

One exception to the trusted-ground-control-station model is if :ref:`MAVLink2 Signing <common-MAVLink2-signing>` is enabled.  If the serial port is configured for MAVLink2 signing, unsigned packets will be ignored.  If you believe you have found a situation where an unsigned packet on a MAVLink2 signed connection causes unexpected behaviour please see the "Reporting Security Issues" section below.

`Mission Planner <https://ardupilot.org/planner/>`__ and `MAVProxy <https://ardupilot.org/mavproxy/>`__ are the primary ArduPilot GCS software packages.  Because they can upload firmware, modify parameters, upload missions and send MAVLink commands they have significant control over the vehicle.  Security recommendations:

- When downloading GCS software, only download from the official ArduPilot links or other official sources
- Keep the GCS Software updated to the latest stable version
- Be cautious when connecting to vehicles of unknown provenance
- Enable :ref:`MAVLink2 Signing <common-MAVLink2-signing>` when connecting over untrusted radio or network links
- On shared computers, be aware that the GCS Software may cache flight logs, connection settings and vehicle parameters
- Ensure that the computer running the GCS Software is secure and free from malware
- Be aware of what networks the computer running the GCS Software is connected to. Ensure that it has appropriate protections (ie firewall) from untrusted networks while the GCS Software is running, as this could potentially allow a malicious actor to interfere with the MAVLink connection
- The GCS Software can forward MAVLink connections to multiple consumers - ensure all connected consumers are trusted. Consider setting MAVLink forwarding as "read-only" if available 
- The GCS Software can run scripts and plugins which can execute arbitrary code - only use trusted scripts and plugins

 `QGroundControl <https://qgroundcontrol.com/>`__ is a popular GCS that supports ArduPilot but is not developed by the ArduPilot team.  The above recommendations apply to QGroundControl as well.

ArduPilot GCS's are designed to be robust against malformed data, and malicious actors gaining control of the vehicle via software bugs or exploits.  If you observe this, please report it via the "Reporting Security Issues" process described below.

Support Libraries
=================

`pymavlink <https://github.com/ArduPilot/pymavlink>`__ is the Python library underlying MAVProxy and many other ArduPilot tools.  It provides direct MAVLink message generation, parsing and signing.  Security recommendations:

- Keep pymavlink updated to the latest version
- When downloading pymavlink, only download from the official ArduPilot Github or PyPI repository
- Use the MAVLink2 signing functionality provided by pymavlink for authenticated connections
- Validate all inputs before constructing and sending MAVLink messages
- Exercise caution when parsing MAVLink logs or messages from untrusted sources
- Don't run pymavlink on untrusted XML message definitions

Also included here are the MAVLink nodejs packages `node-mavlink <https://www.npmjs.com/package/node-mavlink>`__ and `mavlink-mappings <https://www.npmjs.com/package/mavlink-mappings>`__ which are used by some ArduPilot tools.

ArduPilot support libraries are designed to be robust against malformed data, and malicious actors gaining control of the vehicle via software bugs or exploits.  If you observe this, please report it via the "Reporting Security Issues" process described below.

Web Tools
=========

The `ArduPilot WebTools <https://firmware.ardupilot.org/Tools/WebTools/>`__ are a collection of browser-based tools for log analysis, parameter review and filter design.  See :ref:`common-webtools` for a full list of available tools.  These tools run entirely within the user's web browser with no data uploaded to a remote server.  Security recommendations:

- Keep the browser updated to benefit from the latest browser security patches
- Only open log files and parameter files from trusted sources, as they are processed by JavaScript running in the browser
- Be aware that browser extensions can potentially intercept data processed within web applications

Other Web-based tools such as the `Terrain Generator <https://firmware.ardupilot.org/Tools/TerrainGenerator/>`__, `Custom Build Server <https://custom.ardupilot.org/>`__, `DroneCan Web Interface <https://can.ardupilot.org/>`__ and the `ArduPilot Firmware Server <https://firmware.ardupilot.org/>`__ are hosted on secure servers with HTTPS and are maintained by the core dev team to ensure security patches are applied in a timely manner.

Any security issues with these tools should be reported via the "Reporting Security Issues" process described below.

Support Proxy
=============

The `ArduPilot Support Proxy <https://support.ardupilot.org>`__ is a tool that proxies MAVLink connections via a secure server, typically for remote support of ArduPilot vehicles.  It provides features such as MAVLink2 signing and connection logging.  Security recommendations:

- Only connect when required and disconnect when finished
- Use MAVLink signing, preferably bi-directional signing, to ensure the authenticity of the connection
- Ensure you trust the support provider you are connecting to, as they will have control over the vehicle while connected
- Ensure you connect to the genuine ArduPilot Support Proxy server and not a malicious impersonator.  The genuine server is hosted at support.ardupilot.org

Any security issues with this tool should be reported via the "Reporting Security Issues" process described below.

Build the Code Yourself
-----------------------

The `ArduPilot firmware server <https://firmware.ardupilot.org/>`__ provides pre-compiled firmware for a wide range of boards and configurations.  The core dev team diligently maintains the server including applying security patches and updates.  However, if you want to be sure that the code running on your autopilot is exactly the code you have reviewed, consider :ref:`building the code yourself <building-the-code>`.

Other components, such as GCS software or support tools can also be built and run locally.

Reporting Security Issues
-------------------------

If you believe you have found a security issue in ArduPilot, please raise an issue in the `issues list <https://github.com/ArduPilot/ardupilot/issues>`__ for ArduPilot or the relevant repository for other software components and optionally post a link to the issue in the :ref:`ArduPilot Discord Server's <ardupilot-discord-server>` "code_review" channel.

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
