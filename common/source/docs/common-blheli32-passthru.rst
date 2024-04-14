.. _common-blheli32-passthru:

==========================
BLHeli32 and BLHeli_S ESCs
==========================

The BLHeli firmware and configuration applications were developed to allow the configuration of ESCs and provide additional features. ESCs with this firmware allow configuring timing, motor direction, LEDs, motor drive frequency, etc.  Before attempting to use BLHeli please follow the :ref:`DShot setup instructions <common-dshot-escs>`.

Depending on ESC, BLHeli/BLHeli_S/BLHeli32 provides the following features

- Pass-Through support in ArdduPilot allows the BLHeliSuite and other ESC configurators to be used to configure the ESCs while remaining connected to the autopilot via USB cable.
- :ref:`Reversible DShot <blheli32-reversible-dshot>` (aka 3D mode) allows the motor to be spun in either direction
- :ref:`Bi-directional DShot <bidir-dshot>` allows the ESCs to send RPM back to the autopilot without the need for an additional telemetry connection
- :ref:`ESC Telemetry <blheli32-esc-telemetry>` allows the ESCs to send RPM, voltage and current information back to the autopilot so that it can be logged, viewed in real-time or even allow the removal of a :ref:`battery monitor <esc-telemetry-based-battery-monitor>`

"BLHeli" covers covers multiple (sometimes competing) projects providing ESCs firmware and accompanying configuration software

- BLHeli was the original open source software that is no longer maintained and is not available on modern ESCs
- `BLHeli32 <https://github.com/bitdump/BLHeli>`__ is closed source and based on 32bit ARM MCUs.  All modern BLHeli ESCs use BLHeli32
- `BLHeli_S <https://github.com/bitdump/BLHeli>`__ is open source and 16bit.  This is no longer actively maintained but the last published version, 16.7, is installed by default on "BLHeli_S" ESCs when shipped from the factory
- `BLHeli_S JESC <https://jflight.net>`__ is paid, closed source software and 16bit allowing it to run on lower end hardware
- `BLHeli_S BlueJay <https://github.com/mathiasvr/bluejay>`__ is free, open source software and 16bit
- `Web-based ESC Configurator for the above <https://esc-configurator.com/>`__  (due to timing variations, being internet based, reliability of connection thru ArduPilot is variable).

.. note:: firmware 4.5 or later required to use this ESC Configurator tool.

Pass-Through Support
--------------------

..note:: This feature is only available on NON IOMCU outputs. Autopilots which have an IOMCU co-processsor (usually marked as having "MAIN" outputs from the IOMCU and "AUX" outputs from the main cpu) will not pass-through on those outputs. Use this features on "AUX" or "FMU" outputs with DShot capability.

The Pass-Through feature allows BLHeli32 and BLHeli_S ESCs to be upgraded and configured using the corresponding BLHeliSuite32 or BLHeliSuite application (running on the user's PC) while the ESCs remain connected to the autopilot.  To use this feature please follow these steps

- Download and install `BLHeliSuite32 <https://github.com/bitdump/BLHeli/releases>`__ (for use with BLHeli32 ESCs), `BLHeliSuite <https://github.com/bitdump/BLHeli>`__ (for BLHeli_S ESC) or `JESC configurator <https://github.com/jflight-public/jesc-configurator/releases>`__ (for use with BLHeli_S JESC) on your PC
- Connect your PC to the autopilot using a USB cable and then connect with a ground station (e.g. Mission Planner, QGC).
- Set :ref:`SERVO_BLH_AUTO <SERVO_BLH_AUTO>` to 1 to automatically enable pass-through on all outputs configured as motors (e.g. :ref:`SERVOx_FUNCTION <SERVO9_FUNCTION>` = "Motor1", "Motor2", etc) for multicopters and quadplanes or throttle (e.g. those with :ref:`SERVOx_FUNCTION <SERVO9_FUNCTION>` set to 70 ("throttle"), 73 ("throttle left") or 74 ("throttle right")) on rovers.  For most multicopters, quadplanes and rovers this will do the right thing but for planes, set :ref:`SERVO_BLH_MASK <SERVO_BLH_MASK>` to enable pass-through on the appropriate servo outputs.
- If your PC is connected to the autopilot using a telemetry radio (instead of using USB cable as recommended above) set :ref:`SERVO_BLH_PORT <SERVO_BLH_PORT>` to the autopilot port connected to the telemetry radio.  Beware that this is does NOT specify the port used for :ref:`ESC telemetry <blheli32-esc-telemetry>` feedback to your autopilot!
- If using a safety switch ensure it is pushed (or disabled by setting :ref:`BRD_SAFETY_DEFLT <BRD_SAFETY_DEFLT>` = 0).  (``BRD_SAFETYENABLE`` in older firmware versions)
- Disconnect the ground station (but leave the USB cable connected)
- Start the ESC configuration software and connect to the autopilot's COM port by selecting "BLHeli32 Bootloader (Betaflight/Cleanflight)" from the interfaces menu.  Press "Connect" and "Read Setup".  You should be able to upgrade and configure all connected ESCs

  .. image:: ../../../images/blhelisuite32.jpg
    :target: ../_images/blhelisuite32.jpg
    :width: 450px

.. note::
   ArduPilot firmware supports the pass-through protocol with up-to-date BLHeli32 firmware and BLHeliSuite32, or BLHeli_S firmware and BLHeliSuite only.

.. warning::
   For pass-through to function, the autopilot must be configured to use one of the DShot protocols.  If you wish to eventually use one of the other protocols (e.g. PWM, OneShot125) that the ESC supports, you may still configure the ESCs using Pass-Through (e.g. change motor directions, set min/max values, etc) but then finally re-configure the autopilot to *not* use DShot.  Once the autopilot and ESCs are rebooted the ESC should auto-detect that the ESCs are no longer using DShot.

..  youtube:: np7xXY_e5sA
    :width: 100%

