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
- DShot600 at 600kbaud (recommended)
- DShot1200 at 1200kbaud
- Bi-directional DShot at 150, 300, 600 and 1200kbaud on supported firmware

These are selected by the :ref:`MOT_PWM_TYPE <MOT_PWM_TYPE>` parameter on Copter or :ref:`Q_M_PWM_TYPE <Q_M_PWM_TYPE>` on QuadPlanes for copter function motors. On Plane, other motors use Normal (PWM) protocol. Bi-directional DShot is selected by selecting DShot in :ref:`MOT_PWM_TYPE <MOT_PWM_TYPE>` and then selecting the required channels through :ref:`SERVO_BLH_BDMASK <SERVO_BLH_BDMASK>`.

.. note:: The autopilot should be re-booted after changing the protocol type.

However, in Plane, other motors, like the traditional fixed wing's main motor or Dual Motor Tailsitters (SERVOn_FUNCTION = 70 throttle, 73 throttle left and / or 74 throttle right), can be changed to another protocol using the :ref:`SERVO_BLH_MASK<SERVO_BLH_MASK>` parameter to specify the output number of the motor together with the :ref:`SERVO_BLH_OTYPE<SERVO_BLH_OTYPE>` parameter to select the protocol of these motors independently of that selected for the copter function motors. 

.. note:: All mask-based configuration can only be changed at a PWM group level, please consult the documentation for your flight controller to ascertain which outputs are on different groups.

.. warning:: Be sure your ESC can support the configuration you select for it. Damage can occur otherwise. This includes frame rates discussed below. Also be careful when switching between digital and analogue output types without re-calibrating ESCs as this can lead to uncommanded motor output.

PWM ESCs Frame Rates
====================

ArduPilot defaults will work with all PWM ESCs. If the ESC is capable of a higher frame rate than 50Hz then :ref:`RC_SPEED<RC_SPEED>` parameter can be set accordingly. This includes use with OneShot125 ESCs if they support the higher frame rates.

OneShot is a special case. It is Normal PWM, but at a frame rate that is the same as the main loop rate, :ref:`SCHED_LOOP_RATE<SCHED_LOOP_RATE>` .

DShot
=====

The DShot protocol can run at different speeds. ArduPilot supports four speeds:

- DShot150 at 150kbaud (recommended)
- DShot300 at 300kbaud
- DShot600 at 600kbaud (recommended)
- DShot1200 at 1200kbaud

For larger aircraft with longer cable runs we recommend using the lowest baud rate, DShot150, as it is the most reliable protocol (lower baudrates are less susceptible to noise on cables). Higher rates can be more susceptible to noise but also tie up the allocated DMA channel for a shorter period so can be beneficial on flight controllers with a lot of DMA sharing.
For BLHeli32 ESCs - used mainly on smaller craft - DShot600 is by far the most widely used and can therefore be a more suitable choice simply because of the amount of testing that it has had.

Bi-directional DShot involves a longer pulse width since it has to wait for a response from the ESC before it can send another pulse and thus DShot300 and DShot600 are to be preferred. Bi-directional DShot does not share DMA channels and so there is no impact on other peripherals.

.. note:: When an output is configured for DShot, the ``SERVOx_MIN/MAX/TRIM`` parameters for that output will always be ignored since DShot does not use these parameters. The trim  value used will be  1500 if it's a reversible output, or 1000 if normal output setup in DShot, and the output range always be 1000-2000.

The frequency at which DShot pulses are sent can be configured through :ref:`SERVO_DSHOT_RATE<SERVO_DSHOT_RATE>`. By default ArduPilot will output a DShot pulse every time a new IMU sample is ready *and* at a fixed 1Khz interval. On a copter with the standard 400Hz scheduler loop rate this works out at about 1.4Khz. However, the output is quite irregular - in order to get more regular output :ref:`SERVO_DSHOT_RATE<SERVO_DSHOT_RATE>` can be configured to send pulses at multiples of the scheduler loop rate. Thus if set to 2 the pulses will be sent at 800Hz, set to 3 at 1.2Khz and so on. The difference being very, very even output which can benefit copters needing tighter motor control (for instance smaller racers). It is not recommended to send pulses at less than 1Khz due to reports of BLHeli32 occasionally missing frames on some flight controllers, similarly sending at higher rates can result in increased reliability and faster recovery from missed pulses where needed at the cost of some CPU. Very high rates can only be used for faster DShot speeds since otherwise pulses might overlap - for instance the fastest rate that DShot150 can theoretically support is 4Khz.

DShot Commands
--------------

On certain ESCs DShot commands are supported. These allow functions such as ESC LEDs, beeps and motor direction to be manipulated by the flight controller. In order to use DShot commands :ref:`SERVO_DSHOT_ESC<SERVO_DSHOT_ESC>` must be set to the type of ESC that is in use. Notify functions (e.g. LEDs :ref:`NTF_LED_TYPES<NTF_LED_TYPES>` and Buzzer :ref:`NTF_BUZZ_TYPES<NTF_BUZZ_TYPES>`) can then be configured to include DShot as an output type.

The current commands supported are:

-    Dshot LEDs on/off
-    Dshot Buzzer on/off
-    Reverse motor direction
-    Enable 3D operation

.. warning:: Currently, ArduPilot supports the command set (:ref:`SERVO_DSHOT_ESC<SERVO_DSHOT_ESC>` =1) that is commonly used, however, others are appearing and may not be compatible, resulting in undefined operation. Use caution (remove blades!) until correct operation using type=1 is verified for your ESC

Reversible DShot ESCs
---------------------

Currently, only BLHeli32 style reversible DShot ESCs are supported. In order to use one, the output which drives it must be designated with the appropriate bit in the :ref:`SERVO_BLH_3DMASK<SERVO_BLH_3DMASK>` bitmask parameter. This will map the outputs 1000-1500-2000 values to the correct digital values for the ESC to provide FullReverse-Idle-FullForward range operation, respectively.

If the craft has been setup for DShot commands then ArduPilot will supply the correct command at startup in order to set the ESCs in reversible mode.

 In a similar fashion, normal output rotation direction can be reversed by setting :ref:`SERVO_BLH_RVMASK<SERVO_BLH_RVMASK>` without any changes needing to be made through ESC setup software (e.g. BLHeliSuite). This can also be used on ESCs with forward and reversed active operation, ie reversible ESCs, to set the "forward" direction's rotation.

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
