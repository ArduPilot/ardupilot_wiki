.. _common-dshot:

=========================
ESC Related Configuration
=========================


ESC Protocol
============

Ardupilot supports several communication protocols with brushless motor ESCs:

- Normal (standard PWM 1000uS - 2000uS pulse widths)
- Oneshot
- Oneshot125 (PWM pulse widths divided by 8)
- DShot150 at 150kbaud (recommended)
- DShot300 at 300kbaud
- DShot600 at 600kbaud
- DShot1200 at 1200kbaud

These are selected by the :ref:`MOT_PWM_TYPE <MOT_PWM_TYPE>` parameter on Copter or :ref:`Q_M_PWM_TYPE <Q_M_PWM_TYPE>` on QuadPlanes for copter function motors. On Plane, other motors use Normal (PWM) protocol.

.. note:: The autopilot should be re-booted after changing the protocol type.

However,in Plane, other motors,like the traditional fixed wing's main motor or Dual Motor Tailsitters (SERVOn_FUNCTION = 70 throttle, 73 throttle left and / or 74 throttle right), can be changed to another protocol using the :ref:`SERVO_BLH_MASK<SERVO_BLH_MASK>` parameter to specify the output number of the motor together with the :ref:`SERVO_BLH_OTYPE<SERVO_BLH_OTYPE>` parameter to select the protocol of these motors independently of that selected for the copter function motors. 

.. warning:: Be sure your ESC can support the configuraton you select for it. Damage can occur otherwise. This includes frame rates discussed below.

PWM ESCs Frame Rates
====================

ArduPilot defaults will work with all PWM ESCs. If the ESC is capable of a higher frame rate than 50Hz then :ref:`RC_SPEED<RC_SPEED>` parameter can be set accordingly. This includes use with OneShot125 ESCs if they support the higher frame rates.

OneShot is a special case. It is Normal PWM, but at a frame rate that is the same as the main loop rate, :ref:`SCHED_LOOP_RATE<SCHED_LOOP_RATE>` .

DShot
=====

The DShot protocol can run at different speeds. ArduPilot supports four speeds:

- DShot150 at 150kbaud (recommended)
- DShot300 at 300kbaud
- DShot600 at 600kbaud
- DShot1200 at 1200kbaud

We recommend using the lowest baud rate, DShot150, as it is the most reliable protocol (lower baudrates are less susceptible to noise on cables). Higher values will be beneficial once ArduPilot's main loop rate is capable of speeds above 1kHz.

.. note:: When an output is configured for DShot, the ``SERVOx_MIN/MAX/TRIM`` parameters for that output will always be ignored since DShot does not use these parameters. The trim  value used will be  1500 if it's a reversible output, or 1000 if normal output setup in DShot, and the output range always be 1000-2000.

Reversible DShot ESCs
---------------------

Currently, only BLHeli32 style reversible DShot ESCs are supported. In order to use one, the output which drives it must be designated with the appropriate bit in the :ref:`SERVO_BLH_RVMASK<SERVO_BLH_RVMASK>` bitmask parameter. This will map the outputs 1000-1500-2000 values to the correct digital values for the ESC to provide FullReverse-Idle-FullForward range operation, respectively.

.. note:: Currently, ArduPilot only supports the use of reversible ESCs for Plane and Rover, not Copter.

Autopilot Output Capabilities
=============================

While all the servo/motor outputs of an ArduPilot autopilot are capable of Normal PWM operation at 50Hz frame rates, not all are capable of advanced configurations. And, usually, these configurations must apply to pre-designated groups of outputs, even if they are not all driving an ESC. So the following cautions apply:

#. The 8 "MAIN" outputs of autopilots using an IOMCU (like PixHawk and Cube), cannot be used for  protocols other than Normal PWM and OneShot. On these autopilots, only the additional "AUX" outputs can properly support OneShot125 and DShot protocols. If you attempt to set a "MAIN" output to DShot, then normal PWM output will occur, even though it has been set to a DShot protocol.

#. For :ref:`Pixracer <common-pixracer-overview>` and :ref:`other boards <common-autopilots>` without a separate IOMCU coprocessor, all servo/motor outputs can be used.

#. Groups of outputs sharing a common timer, MUST have the same advanced configuration. Usually, these are specified in the autopilot's hardware description linked from the :ref:`common-autopilots` page. For example, if an output is configured for DShot in a group, then you cannot use another output in that group for Normal PWM ESC **or** normal PWM servo operation.

.. note:: Everytime the autopilot initializes, it sends a log message to the ground control station, showing which outputs are PWM/Oneshot/or DShot. The remaining higher numbered outputs are assigned as GPIOs.


.. image:: ../../../images/RCOutbanner.jpg

BLHeli32 Compatibility
======================

Ardupilot is compatible with BLHeli32 Passthrough and ESC Telemetry Functions:

.. toctree::
    :maxdepth: 1

    BLHeli32 Passthrough <common-blheli32-passthru>
    BLHeli32 ESC Telemetry <common-dshot-blheli32-telemetry>
