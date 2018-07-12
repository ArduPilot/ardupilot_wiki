.. _common-pixhawk-overview:

================
Pixhawk Overview
================

Specifications
==============

-  **Processor**

   -  32-bit ARM Cortex M4 core with FPU
   -  168 Mhz/256 KB RAM/2 MB Flash
   -  32-bit failsafe co-processor

-  **Sensors**

   -  MPU6000 as main accel and gyro
   -  ST Micro 16-bit gyroscope
   -  ST Micro 14-bit accelerometer/compass (magnetometer)
   -  MEAS barometer

-  **Power**

   -  Ideal diode controller with automatic failover
   -  Servo rail high-power (7 V) and high-current ready
   -  All peripheral outputs over-current protected, all inputs ESD
      protected

-  **Interfaces**

   -  5x UART serial ports, 1 high-power capable, 2 with HW flow
      control
   -  Spektrum DSM/DSM2/DSM-X Satellite input
   -  Futaba S.BUS input (output not yet implemented)
   -  PPM sum signal
   -  RSSI (PWM or voltage) input
   -  I2C, SPI, 2x CAN, USB
   -  3.3V and 6.6V ADC inputs

-  **Dimensions**

   -  Weight 38 g (1.3 oz)
   -  Width 50 mm (2.0”)
   -  Height 15.5 mm (.6”)
   -  Length 81.5 mm (3.2”)

Pixhawk connector assignments
=============================

.. image:: ../../../images/Pixhawk_with_legend.jpg
    :target: ../_images/Pixhawk_with_legend.jpg

.. image:: ../../../images/pixhawk-status-LEDs-definition.jpg
    :target: ../_images/pixhawk-status-LEDs-definition.jpg

Pixhawk top connectors
======================

.. image:: ../../../images/PixhawkLabled.jpg
    :target: ../_images/PixhawkLabled.jpg

Pixhawk PWM connectors for servos and ESCs and PPM-SUM in and SBUS out
======================================================================

.. image:: ../../../images/pixhawkPWM.jpg
    :target: ../_images/pixhawkPWM.jpg

Pixhawk connector diagram
=========================

.. image:: ../../../images/PixHawk_labelled.png
    :target: ../_images/PixHawk_labelled.png

**For all connectors pin 1 is on the right in the above image**

**Serial 1 (Telem 1) and Serial 2 (Telem 2) Pins: 6 = GND, 5 =
RTS, 4 = CTS, 3 = RX, 2 = TX, 1 = 5V.**

.. _common-pixhawk-overview_pixhawk_connector_pin_assignments:

Pixhawk connector pin assignments
=================================

TELEM1, TELEM2 ports
~~~~~~~~~~~~~~~~~~~~

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin </th>
   <th>Signal </th>
   <th>Volt </th>
   </tr>
   <tr>
   <td>1 (red)</td>
   <td>VCC</td>
   <td>+5V</td>
   </tr>
   <tr>
   <td>2 (blk)</td>
   <td>TX (OUT)</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>3 (blk)</td>
   <td>RX (IN)</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>4 (blk)</td>
   <td>CTS</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>5 (blk)</td>
   <td>RTS</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>6 (blk)</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   </tbody>
   </table>



GPS port
~~~~~~~~

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin</th>
   <th>Signal</th>
   <th>Volt</th>
   </tr>
   <tr>
   <td>1 (red)</td>
   <td>VCC</td>
   <td>+5V</td>
   </tr>
   <tr>
   <td>2 (blk)</td>
   <td>TX (OUT)</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>3 (blk)</td>
   <td>RX (IN)</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>4 (blk)</td>
   <td>CAN2 TX</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>5 (blk)</td>
   <td>CAN2 RX</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>6 (blk)</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   </tbody>
   </table>



SERIAL 4/5 port - due to space constraints two ports are on one connector.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin</th>
   <th>Signal</th>
   <th>Volt</th>
   </tr>
   <tr>
   <td>1 (red)</td>
   <td>VCC</td>
   <td>+5V</td>
   </tr>
   <tr>
   <td>2 (blk)</td>
   <td>TX (#4)</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>3 (blk)</td>
   <td>RX (#4)</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>4 (blk)</td>
   <td>TX (#5)</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>5 (blk)</td>
   <td>RX (#5)</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>6 (blk)</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   </tbody>
   </table>


ADC 6.6V
~~~~~~~~


.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin</th>
   <th>Signal</th>
   <th>Volt</th>
   </tr>
   <tr>
   <td>1 (red)</td>
   <td>VCC</td>
   <td>+5V</td>
   </tr>
   <tr>
   <td>2 (blk)</td>
   <td>ADC IN</td>
   <td>up to +6.6V</td>
   </tr>
   <tr>
   <td>3 (blk)</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   </tbody>
   </table>



ADC 3.3V
~~~~~~~~



.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin</th>
   <th>Signal</th>
   <th>Volt</th>
   </tr>
   <tr>
   <td>1 (red)</td>
   <td>VCC</td>
   <td>+5V</td>
   </tr>
   <tr>
   <td>2 (blk)</td>
   <td>ADC IN</td>
   <td>up to +3.3V</td>
   </tr>
   <tr>
   <td>3 (blk)</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   <tr>
   <td>4 (blk)</td>
   <td>ADC IN</td>
   <td>up to +3.3V</td>
   </tr>
   <tr>
   <td>5 (blk)</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   </tbody>
   </table>



I2C
~~~



.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin</th>
   <th>Signal</th>
   <th>Volt</th>
   </tr>
   <tr>
   <td>1 (red)</td>
   <td>VCC</td>
   <td>+5V</td>
   </tr>
   <tr>
   <td>2 (blk)</td>
   <td>SCL</td>
   <td>+3.3 (pullups)</td>
   </tr>
   <tr>
   <td>3 (blk)</td>
   <td>SDA</td>
   <td>+3.3 (pullups)</td>
   </tr>
   <tr>
   <td>4 (blk)</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   </tbody>
   </table>



CAN
~~~



.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin</th>
   <th>Signal</th>
   <th>Volt</th>
   </tr>
   <tr>
   <td>1 (red)</td>
   <td>VCC</td>
   <td>+5V</td>
   </tr>
   <tr>
   <td>2 (blk)</td>
   <td>CAN_H</td>
   <td>+12V</td>
   </tr>
   <tr>
   <td>3 (blk)</td>
   <td>CAN_L</td>
   <td>+12V</td>
   </tr>
   <tr>
   <td>4 (blk)</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   </tbody>
   </table>

SPI
~~~



.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin</th>
   <th>Signal</th>
   <th>Volt</th>
   </tr>
   <tr>
   <td>1 (red)</td>
   <td>VCC</td>
   <td>+5V</td>
   </tr>
   <tr>
   <td>2 (blk)</td>
   <td>SPI_SCK</td>
   <td>3.3V</td>
   </tr>
   <tr>
   <td>3 (blk)</td>
   <td>SPI_MISO</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>4 (blk)</td>
   <td>SPI_MOSI</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>5 (blk)</td>
   <td>!SPI_NSS</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>6 (blk)</td>
   <td>!GPIO</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>7 (blk)</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   </tbody>
   </table>

POWER
~~~~~

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin</th>
   <th>Signal</th>
   <th>Volt</th>
   </tr>
   <tr>
   <td>1 (red)</td>
   <td>VCC</td>
   <td>+5V</td>
   </tr>
   <tr>
   <td>2 (blk)</td>
   <td>VCC</td>
   <td>+5V</td>
   </tr>
   <tr>
   <td>3 (blk)</td>
   <td>CURRENT</td>
   <td>up to +3.3V</td>
   </tr>
   <tr>
   <td>4 (blk)</td>
   <td>VOLTAGE</td>
   <td>up to +3.3V</td>
   </tr>
   <td>5 (blk)</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   <td>6 (blk)</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   </tbody>
   </table>

SWITCH
~~~~~~

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin</th>
   <th>Signal</th>
   <th>Volt</th>
   </tr>
   <tr>
   <td>1 (red)</td>
   <td>VCC</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>2 (blk)</td>
   <td>!IO_LED_SAFETY</td>
   <td>GND</td>
   </tr>
   <tr>
   <td>3 (blk)</td>
   <td>SAFETY</td>
   <td>GND</td>
   </tr>
   </tbody>
   </table>

Console Port
~~~~~~~~~~~~

The system's serial console runs on the port labeled SERIAL4/5. The
pinout is standard serial pinout, to connect to a standard FTDI cable
(3.3V, but it's 5V tolerant).


.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pixhawk</th>
   <th></th>
   <th>FTDI</th>
   <th>
   </th>
   </tr>
   <tr>
   <td>1</td>
   <td>+5V (red)</td>
   <td>
   </td>
   <td>N/C</td>
   </tr>
   <tr>
   <td>2</td>
   <td>Tx</td>
   <td></td>
   <td>N/C</td>
   </tr>
   <tr>
   <td>3</td>
   <td>Rx</td>
   <td>
   </td>
   <td>N/C</td>
   </tr>
   <tr>
   <td>4</td>
   <td>Tx</td>
   <td>5</td>
   <td>Rx (yellow)</td>
   </tr>
   <tr>
   <td>5</td>
   <td>Rx</td>
   <td>4</td>
   <td>Tx (orange)</td>
   </tr>
   <tr>
   <td>6</td>
   <td>GND</td>
   <td>1</td>
   <td>GND (black)</td>
   </tr>
   </tbody>
   </table>

Spektrum/DSM Port
~~~~~~~~~~~~~~~~~

The Spektrum/DSM port is for connecting Spektrum DSM-2/DSMX receiver
modules.

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Pin</th>
   <th>Signal</th>
   <th>Volt</th>
   </tr>
   <tr>
   <td>1 (white)</td>
   <td>Signal</td>
   <td>+3.3V</td>
   </tr>
   <tr>
   <td>2 (black)</td>
   <td>GND</td>
   <td>GND</td>
   </tr>
   <tr>
   <td>3 (red)</td>
   <td>VCC</td>
   <td>+3.3V</td>
   </tr>
   </tbody>
   </table>


Pixhawk system features
=======================

-  The Pixhawk (FMUv2) flight controller consists of a PX4-FMU controller
   and a PX4-IO integrated on a single board with additional IO, Memory
   and other features.
-  It is highly optimized to provide control and automation for APM
   flight navigation software with high performance and
   capacity. Pixhawk allows users of older boards to seamlessly
   transition to this system and lowers the barriers to entry for new
   users.
-  The NuttX real-time operating system features high performance,
   flexibility, and reliability for controlling any autonomous vehicle.
-  A Unix/Linux-like programming environment, integrated
   multithreading and autopilot functions such as scripting of
   missions and flight behavior provide powerful development
   capabilities.
-  A custom PX4 driver layer ensures tight timing across all processes.
-  Peripheral options include digital airspeed sensors,
   external multi-color LED indicators and external
   compasses.
-  Most peripherals are automatically detected and configured.
-  **A very powerful 32-bit processor with an additional failsafe backup
   controller and extensive memory.**

   -  STM32F427 32-bit primary microcontroller: 168 MHz, 252 MIPS,
      Cortex M4 core with a floating point unit.
   -  Two megabytes of Flash program memory and 256 kilobytes of RAM.
   -  STM32F103 backup failsafe 32-bit co-processor provides for manual
      recovery and has its own power supply.
   -  Socket for a plug in micro SD memory card for data logging and
      other uses.

-  **Advanced sensor profile**

   -  3 axis 16-bit ST Micro L3GD20H gyro for determining orientation.
   -  3 axis 14-bit accelerometer and compass for determining outside
      influences and compass heading.
   -  Provision for external compass with automatic switch-over if
      desired.
   -  MEAS MS5611 barometric pressure sensor for determining altitude.
   -  Built in voltage and current sensing for battery condition
      determination.
   -  Connections for externally-mountable GPS units for
      determining absolute position.

-  **Extensive I/O interfaces with dedicated connectors**

   -  Fourteen PWM servo or ESC speed control outputs.
   -  Five UARTs (serial ports), one high-power capable, 2 with HW flow
      control.
   -  Two CAN I/O ports (one with internal 3.3V transceiver, one on
      expansion connector)
   -  Spektrum DSM / DSM2 / DSM-X® Satellite reciever compatible input:
      Permits use of Spektrum RC Transmitters.
   -  Futaba S.BUS® compatible input and output.
   -  PPM sum signal input.
   -  RSSI (PWM or voltage) input.
   -  I2C and SPI serial ports.
   -  Two 3.3 volt and one 6.6 volt Analog inputs.
   -  Internal microUSB port and external microUSB port extension.

-  **Comprehensive power system with redundancy and extensive
   protection.**

   -  The Pixhawk is supplied with an in line power supply with voltage
      and current sensor outputs.
   -  Ideal diode controller with redundant power supply inputs and
      automatic fail-over.
   -  Servo rail high-power (max. 10V) and high-current (10A+) ready.
   -  All peripheral outputs are over-current protected and all inputs
      ESD protected.
   -  The provided external safety button enables safe motor activation
      / deactivation.
   -  LED status indicators and driver for high brightness external
      multicolored LED to indicate flight status.
   -  High-power, multi-tone piezo audio indicator also informs of
      current flight status.
   -  High performance UBLOX GPS plus external compass in
      protective case available.
   -  Weight: 38g (1.31oz), Width: 50mm (1.96"), Thickness: 15.5mm
      (.613"), Length: 81.5mm (3.21")

Comparison of PX4FMU/PX4IO and Pixhawk
======================================

The new Pixhawk is an evolution of the PX4FMU  and PX4IO modules
and is completely compatible.

-  The PX4FMU and PX4IO stack is very small (the size of an 8 ch RC
   receiver) and very densely packed, Pixhawk has more space, more
   serial ports and more PWM outputs.
-  There are two groups of servo connectors, one main group of 8 outputs
   wired through the backup processor, and an auxiliary group of 6
   outputs directly wired to the main processor.
-  The port labeled "RC" can take normal PPM sum or Futaba S.Bus inputs
   and the port labeled "SB" can read RSSI or output S.Bus to servos.
-  A Spektrum satellite compatible port is on top (labeled SPKT/DSM).
-  The basic operation is the same, and the software is shared.
-  Inside Pixhawk a FMUv2 and an IOv2 do their duties on a single board
   (and developers will find that the software will refer to FMUv2 and
   IOv2)
-  The Pixhawk system has more than 10 times the CPU performance
   and memory of the APM2.x and a lot more as well.
-  14 PWM outputs (Pixhawk) vs. 12 PWM outputs (PX4FMU/PX4IO)
-  All Pixhawk PWM outputs on servo connectors (PX4: 8 on servo, 4 on 15
   pin DF13 connector)
-  5 serial ports vs. 4 (with some double functionality, so only 3 in
   some configurations on old version)
-  256 KB RAM and 2 MB flash vs 192 KB RAM and 1 MB flash (old)
-  Modernized sensor suite (latest generation)
-  High-power buzzer driver (old: VBAT driven, not as loud)
-  High-power multicolor led (old: only external BlinkM support)
-  Support for panel-mounted USB extension (old: not present)
-  Revised, improved power architecture
-  Better protection on all input / output pins against shorts and over
   voltage
-  Better sensing of power rails (internal and external, e.g. servo
   voltage)
-  Support for Spektrum Satellite pairing (needed some manual wiring
   work in v1, but also software-supported)
-  No more solid state relays on v2 (was not really used)
-  Connectors easier to disconnect in case, as the surrounding plastic
   helps to place the fingers correctly
-  Case prevents one-off failure operation of servo connectors
-  The new unit is consirably larger, has the same height, but offers in
   general more handling convenience.
-  External power supply similar to existing 3DR power brick (every unit
   comes with a free module).
-  Both generations offer the same backup / override processor that
   allows failover to manual if the autopilot fails in fixed wing
   setups.
-  For software developers the differences are nicely abstracted in the
   PX4 middleware, and can be sensed / configured at runtime.

Connecting and disconnecting DF13 connectors
============================================

..  youtube:: Kfu8M8t2fWY
    :width: 100%

..  youtube:: TverfQwSdzU
    :width: 100%
	    
.. _common-pixhawk-overview_pixhawk_analog_input_pins:

Pixhawk analog input pins
=========================

This section lists the analog pins available on the Pixhawk. These are
virtual pins, defined in the firmware.

**Virtual Pin 2 and Power connector Pin 4**: power
management connector voltage pin, accepts up to 3.3V, usually attached
to 3DR power brick with 10.1:1 scaling

**Virtual Pin 3 and Power connector Pin 3**: power management connector
current pin, accepts up to 3.3V, usually attached to 3DR power brick
with 17:1 scaling

**Virtual Pin 4 and (No connector Pin)**: VCC 5V rail sensing. This
virtual pin reads the voltage on the 5V supply rail. It is used to
provide the HWSTATUS.Vcc reading that ground stations use to display 5V
status

**Virtual Pin 13 and ADC 3.3V connector Pin 4**: This takes a max of
3.3V. May be used for sonar or other analog sensors.

**Virtual Pin 14 and ADC 3.3V connector Pin 2**: This takes a max of
3.3V. May be used for second sonar or other analog sensor.

**Virtual Pin 15 and ADC 6.6V connector Pin 2**: analog airspeed sensor
port. This has 2:1 scaling builtin, so can take up to 6.6v analog
inputs. Usually used for analog airspeed, but may be used for analog
sonar or other analog sensors.

.. image:: ../../../images/pixhawk_analog_input_pins.jpg
    :target: ../_images/pixhawk_analog_input_pins.jpg

**Virtual Pin 102**: Servo power rail voltage. This is an internal
measurement of the servo rail voltage made by the IO board within the
Pixhawk. It has 3:1 scaling, allowing it to measure up to 9.9V.

**Virtual Pin 103**: RSSI (Received Signal Strength Input) input pin
voltage (SBus connector output pin). This is the voltage measured by the
RSSI input pin on the SBUS-out connector (the bottom pin of the 2nd last
servo connector on the 14 connector servo rail).

This can alternatively serve as SBus out by setting the
``BRD_SBUS_OUT`` parameter ( :ref:`Copter <copter:BRD_SBUS_OUT>`,
:ref:`Plane <plane:BRD_SBUS_OUT>`, :ref:`Rover <rover:BRD_SBUS_OUT>`).

.. image:: ../../../images/pixhawk2.jpg
    :target: ../_images/pixhawk2.jpg

.. _common-pixhawk-overview_pixhawk_digital_outputs_and_inputs_virtual_pins_50-55:

Pixhawk digital outputs and inputs (Virtual Pins 50-55)
=======================================================

The Pixhawk has no dedicated digital output or input pins on its DF13
connectors, but you can assign up to 6 of the "AUX SERVO" connectors to
be digital outputs/inputs. These are the first 6 of the 14 three-pin
servo connectors on the end of the board. They are marked as AUX servo
pins 1 - 6 on the silkscreen as seen above.

To set the number of these pins that are available as digital
inputs/outputs, set the BRD_PWM_COUNT parameter. On Pixhawk this
defaults to 4, which means the first 4 AUX connectors are for servos
(PWM) and the last 2 are for digital inputs/outputs. If you set
BRD_PWM_COUNT to 0 then you would have 6 virtual digital pins and
still have 8 PWM outputs on the rest of the connector.

The 6 possible pins are available for PIN variables as pin numbers 50 to
55 inclusive. So if you have BRD_PWM_COUNT at the default value of 4,
then the two digital output pins will be pin numbers 54 and 55.

In summary:

If BRD_PWM_CNT= 2 then

50 = RC9

51 = RC10

52 = Aux 3

53 = Aux 4

54 = Aux 5

55 = Aux 6

If BRD_PWM_CNT= 4 then

50 = RC9

51 = RC10

52 = RC11

53 = RC12

54 = Aux 5

55 = Aux 6

If BRD_PWM_CNT= 6 then

50 = RC9

51 = RC10

52 = RC11

53 = RC12

54 = RC13

55 = RC14

By default, the pins are digital outputs as outlined above. A digital
pin will instead be a digital input if it is assigned to a parameter
that represents a digital input. For example, setting CAM_FEEDBACK_PIN
to 50 will make pin 50 the digital input that receives a signal from the
camera when a picture has been taken.

.. _common-pixhawk-overview_powering:

Powering
========

The topic :ref:`Powering the Pixhawk <common-powering-the-pixhawk>`
explains both simple and advanced power-supply options for the Pixhawk.


See also
========

.. toctree::
    :maxdepth: 1

    LEDs <common-leds-pixhawk>
    Safety Switch <common-safety-switch-pixhawk>
    Sounds <common-sounds-pixhawkpx4>
    Pixhawk Serial Names <common-pixhawk-serial-names>

[site wiki="planner"]
    Pixhawk Wiring Quick Start <common-pixhawk-wiring-and-quick-start>
    Powering the Pixhawk <common-powering-the-pixhawk>
    Mounting the Flight Controller <common-mounting-the-flight-controller>
    Compatible RC Transmitter and Receiver Systems (Pixhawk) <common-pixhawk-and-px4-compatible-rc-transmitter-and-receiver-systems>
[/site]
