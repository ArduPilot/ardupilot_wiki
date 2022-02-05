.. _common-blheli32-passthru:

BLHeli32/ BLHeli-S Pass-Through Support
=======================================

In addition to offering enhanced capabilities such as reversing and often, telemetry, BLHeli capable ESCs have the ability to have their parameters and features programmed through the autopilot while it is connected to a PC (if the autopilot firmware provides this capability also). This BLHeli pass-through protocol allows you to configure and upgrade your ESCs without having to disconnect them from your vehicle. You can plug a USB cable into your autopilot and run the BLHeliSuite or BLHeliSuite32 software for Windows to configure your ESCs. 

Most BLHeli ESCs can autodetect if PWM or DShot is selected by the motor protocol parameter set by the user. See the configuration section of :ref:`common-brushless-escs` for more information. In addition, some BLHeli32 capable ESCs offer bi-directional capabilities for telemetry reporting over the control link, see the section below.

.. note::
   ArduPilot firmware supports the pass-through protocol with up-to-date BLHeli_32 firmware and BLHeliSuite32, or BLHeli_S firmware and BLHeliSuite only.

.. warning:: For pass-through to function, the :ref:`motor protocol <MOT_PWM_TYPE>` (Copter,Rover) or :ref:`Q_M_PWM_TYPE<Q_M_PWM_TYPE>` (QuadPlane) must be set to a digital protocol, ie. one of the DShot protocols. If you wish to use one of the other protocols, just reset the motor protocol after using pass-through to change motor directions or set min/max values. The autopilot must be re-booted after a protocol change. In addition, the safety switch, if used, must be pressed to allow the servo/motor outputs to be active.

BLHeli pass-through setup and use:
----------------------------------

..  youtube:: np7xXY_e5sA
    :width: 100%


To enable BLHeli pass-through you need to set the following parameters and reboot your autopilot:

- Set :ref:`SERVO_BLH_AUTO <SERVO_BLH_AUTO>` to 1 to enable automatic mapping of multirotor motors for BLHeli pass-through and telemetry support. for most multirotor and quadplane users this will do the right thing. if using BLHeli ESCs on non-multirotor motors with the respective SERVOn_FUNCTION set to 70 (=throttle), 73 (=throttle left) or 74 (=throttle right), you will need to further specify the used outputs as follows:

- Use :ref:`SERVO_BLH_MASK <SERVO_BLH_MASK>` to enable BLHeli pass-through and telemetry support on non-multirotor motors and / or exactly specify which servo outputs you want to enable BLHeli pass-through and telemetry on.

- Set :ref:`SERVO_BLH_PORT <SERVO_BLH_PORT>` to specify the autopilot's port used to connect to your PC running BLHeliSuite32 (or BLHeliSuite for BLHeli_S) for ESC configuration. It defaults to USB and likely does not need to be altered. Beware that this does NOT specify the serial port used for the ESC's telemetry feedback to your autopilot!

Now connect a USB cable to your autopilot and use BLHeliSuite32 on Windows to connect. Select "BLHeli32 Bootloader (Betaflight/Cleanflight)" from the interfaces menu.

.. image:: ../../../images/blhelisuite32.jpg
    :target: ../_images/blhelisuite32.jpg

Reversible DShot ESCs
---------------------

Currently, only BLHeli32 and BLHeli-S capable reversible DShot ESCs are supported. In order to use one, the output which drives it must be designated with the appropriate bit in the :ref:`SERVO_BLH_3DMASK<SERVO_BLH_3DMASK>` bitmask parameter. This will map the outputs 1000-1500-2000 values to the correct digital values for the ESC to provide FullReverse-Idle-FullForward range operation, respectively.

If the craft has been setup for DShot commands then ArduPilot will supply the correct command at startup in order to set the ESCs in reversible mode.

In a similar fashion, normal output rotation direction can be reversed by setting :ref:`SERVO_BLH_RVMASK<SERVO_BLH_RVMASK>` without any changes needing to be made through ESC setup software (e.g. BLHeliSuite). This can also be used on ESCs with forward and reversed active operation, ie reversible ESCs, to set the "forward" direction's rotation.

.. note:: Currently, ArduPilot only supports the use of reversible ESCs for Plane and Rover, not Copter.

BLHeli32 ESC Telemetry
----------------------

Many brushless BLHeli ESCs offer telemetry reporting of important ESC data, such as RPM, Temperature, Current, etc. for use in OSDs or for controlling ArduPilot functions like variable center frequency noise filters. This can be via a serial connection to one of the autopilot's UART RX inputs, whose ``SERIALx_PROTOCOL`` has been set to "16" (ESC Telemetry), or directly over its control connection if it has Bi-Directional DShot capability (only available on certain BLHeli32 capable ESCs at present).

.. image:: ../../../images/dshot-telemwire.png
    :target: https://shop.holybro.com/holybro-tekko32-esc35a_p1074.html

*image courtesy of holybro.com*

For BLHeli32 ESC telemetery, see:

.. toctree::
    :maxdepth: 1

    BLHeli32 ESC Telemetry <common-dshot-blheli32-telemetry>

