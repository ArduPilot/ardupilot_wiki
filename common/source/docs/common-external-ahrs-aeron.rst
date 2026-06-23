.. _common-external-ahrs-aeron:

==========================
Aeron PLX3-N External AHRS
==========================

`Aeron Systems <https://www.aeronsystems.com/>`__ designs and manufactures miniature MEMS-based Inertial Navigation Systems (INS) for unmanned aerial
and ground vehicles. The Aeron PLX family combines a tri-axial accelerometer,tri-axial gyroscope, magnetometer, barometric pressure sensor and a
multi-constellation GNSS receiver, all fused on-device by an Extended Kalman Filter that produces a complete position-velocity-time-attitude (PVTA)
navigation solution that ArduPilot can use directly.

What you Get
============

* Higher accuracy heading, pitch and roll estimates with tactical and consumer grade sensors.
* Continued navigation through GNSS outages using inertial dead-reckoning with optional airspeed and external-magnetometer aiding.
* Multi-constellation (GPS, GLONASS, Galileo, BeiDou, NavIC) tracking with built-in jamming and spoofing detection.
* A compact, low-power package suitable for size and weight constrained platforms.

ArduPilot can use the PLX3 as an :ref:`external AHRS <common-external-ahrs>`, replacing the EKF3 estimator with the PLX3's onboard navigation solution, or as a supplementary source of GNSS, barometer, and compass data to the EKF3.

The driver supports the **PLX3-N** (Pollux 3 N Micro-Miniature INS) and is expected to work without changes on other members of the PLX family.

For performance specifications, product data sheets, please contact `Aeron Systems <https://www.aeronsystems.com/contact>`__.
Here's a detailed report of GNSS Denied Navigation using PLX3 integrated with Ardupilot. Don't forget to watch the full flight video in GNSS Denied !! `Aeron Systems <https://www.aeronsystems.com/articles/plx3-autopilot-for-autonomous-uav-gnss-denied-operations/>`__.

Before you Start
================

Setting up the PLX3 with ArduPilot requires preparing the device itself and then connecting it to the autopilot. The PLX3 is configured **before** it ever talks to ArduPilot, using the Aeron's **Aeron INS Connect Suite (AICS)** PC tool. ArduPilot's driver is
passive. There is no autopilot-side configuration push at boot.

What this means in practice:

* You need to run AICS at least once, with the PLX3 powered and connected to a Windows/Linux PC, to enable the right binary outputs and set the output rates. Without this step, the autopilot will not receive anything.
* You can re-run AICS at any time to change output rate, calibrate the magnetometer, set a non-default mounting orientation, or enable optional aiding sensors. These changes persist across power cycles.

The PLX3-N User Manual describes AICS in detail (section 8).

Step 1 - Mounting and Antenna
=============================

The PLX3 uses a right-handed coordinate system with **X pointing forward (out of the 16-pin connector face)**, Y to the right, and Z down. The default orientation is connector-forward, base-down. To visualize the system, extend the thumb, index finger, and middle finger of your right hand so they are all perpendicular to each other.
When your thumb aligns with the positive X-axis and your index finger aligns with the positive Y-axis, your middle finger will inevitably point along the positive Z-axis.

Mount the unit on a flat, vibration-free surface using four M2 x 10 mm bolts with M2 spring washers. An optional ø1.5 mm dowel pin can be used to achieve ±0.15° yaw alignment accuracy. In high-vibration installations, isolate the unit from the airframe
with vibration dampers.

If you cannot mount the PLX3 in the default orientation, set the **User Reference Frame Rotation (URFR)** in AICS to compensate. After any change in mounting or installation environment, re-calibrate the internal magnetometer using AICS.

Mount the GNSS antenna rigidly with an unobstructed sky view. A multi-constellation antenna is already provided with INS module; it connects to the PLX3 via the side-mounted MMCX connector.

Step 2 - Wiring to the Autopilot
================================

The PLX3 exposes both a 3.3 V TTL UART and an RS-232 port on its 16-pin interface connector. Pick one and connect it to any spare serial port on the flight controller.

**TTL UART** (recommended for most flight controllers):

  ====================  =====================
  PLX3 pin               Function
  ====================  =====================
  5  (``UART_RX``)      Connect to FC TX
  6  (``UART_TX``)      Connect to FC RX
  16 (``GND_D``)        Signal ground
  ====================  =====================

**RS-232** (needs an external level shifter — most Flight Controllers do not have RS-232 transceivers):

  ====================  =====================
  PLX3 pin               Function
  ====================  =====================
  3  (``RS232_R1IN``)   RS-232 receive
  4  (``RS232_T1OUT``)  RS-232 transmit
  1  (``VIN``)          5 ±0.5 V power
  16 (``GND_D``)        Signal ground
  ====================  =====================

The full 16-pin pinout is in the PLX3-N User Manual.

Step 3 - PLX3-side Configuration (AICS)
=======================================

Unlike some other External AHRS devices, the PLX3 is **configured via the Aeron INS Connect Suite (AICS) PC tool prior to use, not by ArduPilot at runtime**. The driver is passive.

Connect the PLX3 to a Windows/Linux PC via USB-to-UART or USB-to-RS-232, launch AICS, and configure the following. If you have any problems with configuration or need custom settings please reach out to us `Aeron Systems <support.ins@aeronsystems.com>`__.
The following AICS settings must be configured before connecting the PLX3 to ArduPilot:

* **Output Interface** - set to ``UART`` or ``RS232`` depending on the wiring chosen above. The output interface must match the physical connection.
* **Baud Rate** - 921600 baud is recommended. The default of 460800 also works but limits the maximum packet rates.
* **Data Format** - ``Binary``. The ArduPilot driver does not parse the NMEA output.
* **Output rate** for all binary packets that are relevant should be turned on. A starting point of 50 Hz works for most fixed-wing platforms; 
  Copter or QuadPlane installations using the PLX3 IMU should set this to match the vehicle's ``SCHED_LOOP_RATE``.
* **Binary Strings (output enable)** - the following packets must all be enabled:

  * ``NAV_PARA1`` - fused position, velocity and attitude.
  * ``NAV_PARA2`` - quaternion, body velocity, ECEF position, hardware status.
  * ``SENS_PARA`` - raw gyroscope, accelerometer, magnetometer and barometer.
  * ``GPS_PARA`` - GNSS position, fix status and DOP.
  * ``EXTD_GNSS`` - GNSS accuracy estimates and NED velocity.

* **Binary Data Rate** - all packets above must be configured as per specific vehicle chosen. The recommended setting depends on the vehicle type and on whether IMU data will be consumed by ArduPilot 
  (See :ref:`use_as_external_sensor_set` below).

* **User Reference Frame Rotation (URFR)** - configure to match the physical mounting if not using the default connector-forward, base-down orientation.

* **Magnetometer Calibration** - perform after final installation using the AICS calibration workflow.

After setting these parameters, the configuration must be written to the device's using **Save Configuration** in AICS so that it persists across power cycles.

For detailed AICS instructions, refer to the PLX3-N User Manual, section 8 (Aeron INS Connect Suite).

Optional features
-----------------

The PLX3 supports several optional features that are surfaced through the ArduPilot driver:

* **External magnetometer aiding** (via I2C-2) - provides an absolute heading reference in GNSS-denied conditions. Must be enabled in AICS Sensor Settings.
* **External airspeed aiding** (via I2C or SPI) - improves dead-reckoning accuracy in GNSS-denied conditions for fixed-wing platforms. Must be enabled in AICS Sensor Settings.
* **Jamming and spoofing detection** - continuously monitored by the PLX3. Normally, the unit operates on multi-band, multi-constellation if configured, then automatically switches to inertial-only navigation on detection. 
  The detection state is reported to ArduPilot via the ``GPS_PARA`` status word and surfaced as GCS warnings.

Step 4 - ArduPilot Configuration
================================

There are two ways for the PLX3 data to be consumed by ArduPilot: as a supplementary external sensor set feeding ArduPilot's EKF3, or as a full external AHRS replacing the EKF3 entirely.

To establish communication with the PLX3, set the following for the relevant serial port and configuration:

* :ref:`SERIALx_PROTOCOL<SERIAL1_PROTOCOL>` = 36 (AHRS)
* :ref:`SERIALx_BAUD<SERIAL1_BAUD>` = Matching PLX3 Baud Rate (921 for 921600)
* :ref:`EAHRS_TYPE<EAHRS_TYPE>` = 10 (Aeron)
* :ref:`GPS1_TYPE<GPS1_TYPE>` = 21 (External AHRS)

.. tip::

   The full External AHRS-specific parameters may not be visible before the ``EAHRS_TYPE`` parameter is configured. A **Refresh Params** or a reboot of ArduPilot may be necessary to see them.

.. _use_as_external_sensor_set:

Step 5 - Choose how ArduPilot consumes PLX3 data
================================================

There are two ways for ArduPilot to use the PLX3. You can chose between the two freely — the choice is a single parameter at runtime.

Mode A: Supplement ArduPilot's EKF3 with PLX3 sensors
-----------------------------------------------------

If set up as an external sensor, the PLX3's GNSS, barometer and compass data feed ArduPilot's internal EKF3, which continues to be the canonical navigation source. After the serial parameters have
been configured, set:

* :ref:`AHRS_EKF_TYPE<AHRS_EKF_TYPE>` = 3 (ArduPilot's EKF3)

By default the driver advertises ``GPS``, ``BARO``, and ``COMPASS`` as the available sensors. IMU data is **not** offered by default because typical PLX3 ``SENS_PARA`` rates (50-200 Hz) are below the ArduPilot ``SCHED_LOOP_RATE`` (400 Hz Copter, 300 Hz VTOL, 50 Hz Plane) and
would starve the EKF predict step. If your AICS configuration outputs ``SENS_PARA`` at a rate equal to or greater than the vehicle's ``SCHED_LOOP_RATE``, you can opt into using PLX3 IMU data by setting :ref:`EAHRS_SENSORS<EAHRS_SENSORS>` to include the ``IMU`` bit.

When IMU data is consumed, set :ref:`EAHRS_RATE<EAHRS_RATE>` to match the actual ``SENS_PARA`` rate configured in AICS. ``EAHRS_RATE`` must be at least the vehicle's ``SCHED_LOOP_RATE``. An ArduPilot reboot is required after changing ``EAHRS_RATE``.

Mode B: Use the PLX3 as the canonical External AHRS
---------------------------------------------------

Configuring ArduPilot to use the PLX3 as an External AHRS will use the PLX3's onboard PVTA solution as canonical, rather than running the EKF3. This makes use of the PLX3's combined IMU-and-GNSS estimation, which can give higher attitude accuracy and better dynamic performance.

After the common parameters have been configured, set:

* :ref:`AHRS_EKF_TYPE<AHRS_EKF_TYPE>` = 11 (External AHRS)

.. tip::

   ArduPilot's internal EKF3 continues to run alongside the PLX3 even when the PLX3 is the canonical source. It is therefore recommended to additionally configure the PLX3 as an external
   sensor set (per the section above) so that you can switch between the two solutions by changing ``AHRS_EKF_TYPE``, without re-flashing or re-setting.

GNSS-Denied Operation
=====================

In a GNSS-denied environment, the PLX3 continues to provide a position/velocity/attitude solution by dead reckoning from its internal IMU, optionally aided by an external airspeed sensor or external magnetometer. The position uncertainty reported by the
PLX3 grows over time during dead reckoning; this is reflected in the variances the driver reports to the EKF.

DataFlash Logs
==============

The driver writes up to three streaming log messages.

The **AERN** (Aeron Navigation) message contains the fused navigation solution from ``NAV_PARA1``:

* Time (microseconds), Unix epoch seconds
* Latitude, Longitude
* Height above WGS84 ellipsoid
* Velocity North, East, Down
* Course over ground
* Roll, Pitch, Yaw
* PLX3 INS status word, hardware status word

The **AERS** (Aeron Sensors) message contains raw sensor data from ``SENS_PARA``:

* Time (microseconds)
* Sensor temperature
* Gyroscope X/Y/Z
* Accelerometer X/Y/Z
* Magnetometer X/Y/Z
* Barometric pressure, temperature, altitude

The **AERG** (Aeron GNSS) message contains GNSS solution status and accuracy estimates from ``GPS_PARA`` and ``EXTD_GNSS``:

* Time (microseconds), GPS time of week, GPS week number
* Number of satellites in view, fix type
* Jamming and spoofing detection levels
* Horizontal/vertical position accuracy, horizontal velocity accuracy
* Horizontal and vertical dilution of precision

Use these for post-flight tuning and to verify that the rates and accuracies you configured in AICS are what the autopilot is actually receiving.

Troubleshooting
===============

**The autopilot reports "Aeron: Serial port not found".**
The ``SERIALx_PROTOCOL`` for the port you wired is not set to ``36``.
Set it and reboot.

**No ``EAHRS_TYPE`` dropdown does not list "Aeron".**
Your firmware build does not include the Aeron driver. Either grab a firmware from the `Custom Build Server
<https://custom.ardupilot.org/add_build>`__ with External AHRS support enabled, or use an official build from 4.7 onwards.

**Pre-arm fails with "Aeron Unhealthy".**
The driver has not received a complete packet set within the staleness deadlines. Check that all five binary packets are enabled in AICS and that the autopilot baud rate matches the PLX3 baud rate.

**CRC mismatch messages on the GCS.**
Almost always a wiring or baud-rate problem. Double-check ``SERIALx_BAUD`` matches AICS, and verify the TX/RX pins are not swapped.

**The GCS / AERG Log shows ``Jam`` or ``Spf`` greater than 1.**
The PLX3 is reporting GNSS interference. This is informational, not a fault — the PLX3 is doing its job. Check your operating environment for sources of interference.