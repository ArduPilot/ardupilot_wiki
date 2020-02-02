.. _common-blheli32-passthru:

BLHeli_32 Pass-Through Support
==============================

.. note::
   ArduPilot firmware supports the pass-through protocol with up-to-date BLHeli_32 firmware and BLHeliSuite32 only.

BLHeli_32 pass-through protocol allows you to configure and upgrade your ESCs without having to disconnect them from your vehicle. You can plug a USB cable into your autopilot and run the BLHeliSuite32 software for Windows to configure your ESCs. ArduPilot firmware supports the pass-through protocol with BLHeli_32 only.

The following section shows how to setup BLHeli_32 pass-through support:
------------------------------------------------------------------------

..  youtube:: np7xXY_e5sA
    :width: 100%


To enable BLHeli_32 pass-through you need to set the following parameters and reboot your autopilot:

- Set :ref:`SERVO_BLH_AUTO <SERVO_BLH_AUTO>` to 1 to enable automatic mapping of multirotor motors for BLHeli_32 pass-through and telemetry support. for most multirotor and quadplane users this will do the right thing. if using BLHeli_32 ESCs on non-multirotor motors with the respective SERVOn_FUNCTION set to 70 (=throttle), 73 (=throttle left) or 74 (=throttle right), you will need to further specify the used outputs as follows:

- Use :ref:`SERVO_BLH_MASK <SERVO_BLH_MASK>` to enable BLHeli_32 pass-through and telemetry support on non-multirotor motors and / or exactly specify which servo outputs you want to enable BLHeli_32 pass-through and telemetry on.

- Set :ref:`SERVO_BLH_PORT <SERVO_BLH_PORT>` to specify the autopilot's port used to connect to your PC running BLHeliSuite32 for ESC configuration. It defaults to USB and likely does not need to be altered. Beware that this does NOT specify the serial port used for the ESC's telemetry feedback to your autopilot!

Now connect a USB cable to your autopilot and use BLHeliSuite32 on Windows to connect. Select "BLHeli32 Bootloader (Betaflight/Cleanflight)" from the interfaces menu.

.. image:: ../../../images/blhelisuite32.jpg
    :target: ../_images/blhelisuite32.jpg