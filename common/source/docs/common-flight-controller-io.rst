.. _common-flight-controller-io:

============================
Autopilot Inputs and Outputs
============================

All ArduPilot Autopilots provide inputs and outputs for connecting:

- Control Inputs (Mandatory for vehicle operation):

 - Radio Control Receivers
 - MAVLink Data Streams, ie ground control stations or companion computers

- Sensor Inputs:

 - GPS (mandatory item)
 - Compass (mandatory except for certain Plane uses)
 - Airspeed
 - Rangefinders
 - Redundant barometers, IMUs, etc.

- Power Management Unit Inputs (Mandatory to supply power to autopilot)

-  Analog Inputs:

  - Received Signal Strength Input (RSSI)
  - Analog Airspeed Sensors

- Flight Control Outputs (Mandatory for vehicle operation):

 - ESCs for motors
 - Servos for control surfaces/mechanics

- Telemetry Outputs

- Actuators and General Purpose I/O:

 - Relays
 - LEDs
 - Safety Switch
 - Buzzers

.. image:: ../../../images/fc-io.jpg


I/O Port Types
==============

.. note:: See the individual :ref:`autopilot's<common-autopilots>`  description page for what specific ports are provided and exact pinouts of each port.


USB
---

USB is provided as the primary port for configuration of the autopilot. It always appears as the first Serial port (0) to the autopilot. See :ref:`Serial Port Configuration <common-serial-options>` 

UART
----

UART (Universal Asynchronous Receiver/Transmitter) ports are commonly provided by autopilots to connect peripherals such as telemetry radios, GPS receivers, rangefinders, radio modems, and even SBUS servos. Additionally, starting from firmware version 4.0, serial RC receiver inputs can be connected to any UART. See :ref:`Serial Port Configuration <common-serial-options>`. Usually, several UART ports are provided. 

By default, UART connections are point-to-point and do not support bus-style networking. Each UART line directly connects two devices. The connecting cable is typically straight (1:1), meaning no crossing is performed within the cable itself. Therefore, the peripheral devices must internally cross RX/TX signals as required.

Signals
+++++++

.. list-table:: UART Signals
   :header-rows: 1
   :widths: 15 20 15 20 15 15

   * - Signal
     - Description
     - Autopilot Pin
     - Peripheral Pin
     - Wire Color (Pixhawk)
     - Wire Color (ThunderFly)
   * - +5V
     - Power supply (5V)
     - +5V
     - +5V
     - Red
     - Red
   * - TX
     - Data output from autopilot
     - TX
     - RX
     - Black
     - White
   * - RX
     - Data input to autopilot
     - RX
     - TX
     - Black
     - Green
   * - CTS
     - Clear To Send, autopilot ready to receive data
     - CTS (optional)
     - RTS (optional)
     - Black
     - Blue
   * - RTS
     - Request To Send, autopilot ready to send data
     - RTS (optional)
     - CTS (optional)
     - Black
     - Yellow
   * - GND
     - Ground reference
     - GND
     - GND
     - Black
     - Black

CTS (Clear To Send) and RTS (Request To Send) signals form a hardware handshake mechanism that improves reliability by preventing data overflow. If the peripheral does not utilize these signals, they can typically remain disconnected.

Cable Recommendations
+++++++++++++++++++++

UART signals, particularly at higher baud rates, can generate electromagnetic interference (EMI). To minimize EMI:

- Keep UART cable lengths as short as practical.
- Separate UART cables from high-power wires and sensitive sensors, such as GPS receivers and magnetometers.

Unlike twisted-pair cables used for differential protocols (CAN, I2C), cable twisting is generally not applicable for UART signals due to their single-ended nature.


I2C
---

The I2C bus is widely used by autopilots to connect low-bandwidth peripheral components such as magnetometers (compasses), digital airspeed sensors, rangefinders, tachometers, and other sensors. Multiple devices can be connected to a single I2C bus, each identified by a unique address. I2C employs a two-wire open-drain communication method using Serial Clock (SCL) and Serial Data (SDA) lines. The bus lines are typically pulled up to a positive voltage (3.3V or 5V) through resistors, ensuring a default idle state at logic high.

Signals
+++++++

.. list-table:: I2C Signals
   :header-rows: 1
   :widths: 15 25 20 20 10 10

   * - Signal
     - Description
     - Autopilot Pin
     - Peripheral Pin
     - Wire Color (Pixhawk)
     - Wire Color (ThunderFly)
   * - +5V
     - Power supply (5V)
     - +5V
     - +5V
     - Red
     - Red
   * - SCL
     - Serial Clock Line
     - SCL
     - SCL
     - Black
     - Yellow
   * - SDA
     - Serial Data Line
     - SDA
     - SDA
     - Black
     - Green
   * - GND
     - Ground reference
     - GND
     - GND
     - Black
     - Black

Cable Recommendations
+++++++++++++++++++++

Proper cable handling is mandatory for reliable I2C communication, as inappropriately routed I2C signals could be prone to electromagnetic interference (EMI) and crosstalk.

- Keep cables as short as possible.
- Use twisted pairs to reduce crosstalk:
  - Twist each pair (SCL/+5V and SDA/GND) about 10 turns per 30 cm.
  - Twist both pairs together about 4 turns per 30 cm.
- For larger vehicles or long cables, consider using CAN bus or other differential signaling interfaces instead.

Pull-up Resistors
+++++++++++++++++

Pull-up resistors on both SDA and SCL lines are essential. Autopilots or peripheral devices typically have these resistors built-in. An oscilloscope measurement is sometimes required to check correct value of pull-up resistors. If the signal amplitude is too low, increase resistor values. If the signals are rounded, decrease resistor values.

Common Problems
+++++++++++++++

- **Address Clashes**: Occur when multiple devices on the bus share the same address, preventing proper communication. Possible solutions include:
  - Changing the device address via hardware or software configuration, if supported.
  - Using address translators, such as the `ThunderFly TFI2CADT01 <https://docs.thunderfly.cz/avionics/TFI2CADT01/>`_, which allows multiple devices with identical addresses to coexist by remapping their addresses.

- **Excessive Wiring Capacitance**: Long cables or multiple devices connected to a single bus significantly increase cable capacitance, resulting in degraded signal quality. Possible solutions include:
  - Reducing cable length or using higher-quality cables.
  - Splitting devices across multiple I2C buses.
  - Employing I2C bus accelerators or extenders, such as the `ThunderFly TFI2CEXT01 <https://docs.thunderfly.cz/avionics/TFI2CEXT01/>`_, to boost signal quality over extended cable lengths.

I2C Bus Accelerators and Translators
++++++++++++++++++++++++++++++++++++

- **I2C Bus Accelerators** (e.g., `ThunderFly TFI2CEXT01 <https://docs.thunderfly.cz/avionics/TFI2CEXT01/>`_) physically divide the bus into segments, amplifying signals and reducing the impact of wiring capacitance, thus improving signal integrity and reliability on longer cables. The TFI2CEXT01 can also perform voltage level translation between 3.3V and 5V logic levels.
- **I2C Address Translators** (e.g., `ThunderFly TFI2CADT01 <https://docs.thunderfly.cz/avionics/TFI2CADT01/>`_) allow multiple identical devices with the same I2C address to be connected by remapping device addresses dynamically, resolving conflicts and simplifying sensor integration.


GPS
---

:ref:`GPS<common-positioning-landing-page>` is usually attached to one of the UART ports, but some autopilots provide a connector dedicated to GPS and/or GPS/Compass which includes the I2C signals.

Signals
+++++++

- TX: Data Output, connected to the GPS RX input
- RX: Data Input, connected to the GPS TX output

.. note:: Note the swapping of signals between autopilot and peripheral.

usually +5V and GND are provided in the connector. If its a GPS/Compass port, then the I2C signals will also be provided and attached to the same named signals on the compass sub-module of a GPS/Compass module.


PMU
---

Most autopilots provide the means to attach to a Power Management Unit (PMU) of some kind. These :ref:`units<common-powermodule-landingpage>` provide any, or all, of the following:

- A regulated +5V supply for the autopilot from the flight battery
- Monitoring of current from the flight battery
- Monitoring of voltage from the flight battery

Signals
+++++++

- +5V: Regulated supply to autopilot
- CUR: Current Monitor output. Usually a 0-3.3v analog voltage represents current draw level
- VLT: Voltage Monitor output. Usually a  0-3.3V analog voltage representing battery voltage
- GND: Ground

Some "smart" battery/power monitors replace the CUR and VLT pins with I2C signals to provide digital information on battery status.

Many board style autopilots fully integrate the PMU as part of the board with internal connections to the processor.

In addition, many autopilots offer multiple PMU connections since ArduPilot firmware versions 4.0 and later support up to 10 PMUs/Battery Monitors.

RCIN
----

Input from the radio control receiver is input on this pin. Most serial RC protocols (PPM, SBUS, DSM, etc.) are supported by ArduPilot and auto-detected. In addition, some autopilots provide dedicated connectors for DSM protocol satellite receivers which provide power to the receiver in addition to the input signal.

As of firmware versions 4.0 and later, ArduPilot also allows an RC receiver to be attached to any UART port.

MAIN/AUX/OUT
------------

The primary outputs for controlling motors (via ESCs) and servos are provided by these pins/connectors. They are labeled either as MAIN/AUX outputs or just as OUTPUTs. These outputs provide the PWM or Dshot signals for motor ESC or servo control of flight surfaces. They can also be sometimes used as general purpose I/O pins for controlling relays, parachutes, grippers, etc.

Those controllers with MAIN/AUX output labels usually indicate that a IOMCU co-processor is being employed. These provide outputs intended for use as the motor/servo outputs and provide a redundant means of control via RC if the main autopilot fails. The MAIN outputs come from this co-processor, while the AUX designated outputs are controlled directly from the autopilot. Most board level autopilots do not use an IOMCU and have outputs only labeled OUTPUTx or Mx.

This distinction is important, since AUX outputs(and OUTPUTs from autopilots without an IOMCU) can be used as GPIOs as well as PWM or Dshot. While MAIN outputs can only be used for PWM, except for use as a RELAY GPIOs. See :ref:`GPIOs <common-gpios>`

.. note:: A few autopilots that do NOT use an IOMCU label their outputs as MAIN, so actually do have the capability of use as GPIOs and/or Dshot ESC control on these outputs. CUAV V5 Nano and Holybro Pixhawk 4 Mini are examples.

Often these outputs are provided on 3 pin connector strips supplying or distributing servo power and ground, in addition to the individual output signals. This power is usually provided externally, such as by the ESC or a BEC, although some autopilots provide this power from internal regulators.

CAN
---

:ref:`CAN <common-canbus-setup-advanced>` bus is provided on many autopilots for use with :ref:`DroneCAN <common-uavcan-setup-advanced>` peripherals. ESCs, GPS/Compass, Rangefinders, and many other peripherals and sensors are being added to the list of available DroneCAN devices everyday. CAN provides a robust method of communicating with peripherals with data integrity, even with long leads.


Signals
+++++++

Power and ground lines are typically provided alongside the CAN signals on standard 4-pin connectors (e.g., JST-GH).

.. list-table:: CAN Signals
   :header-rows: 1
   :widths: 15 25 20 20 10 10

   * - Signal
     - Description
     - Autopilot Pin
     - Peripheral Pin
     - Wire Color (Pixhawk)
     - Wire Color (ThunderFly)
   * - +5V
     - Power supply (5V)
     - +5V
     - +5V
     - Red
     - Red
   * - CAN_H
     - CAN high differential signal
     - CAN_H
     - CAN_H
     - Black
     - White
   * - CAN_L
     - CAN low differential signal
     - CAN_L
     - CAN_L
     - Black
     - Yellow
   * - GND
     - Ground reference
     - GND
     - GND
     - Black
     - Black

Cable Recommendations
+++++++++++++++++++++

CAN cables should use twisted pairs to reduce electromagnetic interference (EMI) and maintain signal integrity:

- Twist each signal pair (CAN_H/CAN_L and +5V/GND) approximately 10 turns per 30 cm.
- Twist both pairs together about 4 turns per 30 cm.
- Maintain separation from high-power and high-noise cables.

Due to its differential signaling and robust protocol, CAN is particularly suitable for applications requiring reliability over longer cable lengths and in electrically noisy environments.

SPI
---

SPI (Serial Peripheral Interface) is a synchronous serial communication protocol used by autopilots to connect higher-bandwidth peripherals and sensors, such as optical flow sensors, specialized telemetry modems, barometers, IMUs, and other advanced digital sensors. It supports full-duplex communication using separate lines for data input and output. Most autopilots have processors with multiple SPI ports. 

Signals
+++++++

.. list-table:: SPI Signals
   :header-rows: 1
   :widths: 15 25 20 20 10 10

   * - Signal
     - Description
     - Autopilot Pin
     - Peripheral Pin
     - Wire Color (Pixhawk)
     - Wire Color (ThunderFly)
   * - +5V
     - Power supply (5V)
     - +5V
     - +5V
     - Red
     - Red
   * - SCK
     - Serial Clock, synchronizes data transfer
     - SCK
     - SCK
     - Black
     - Yellow
   * - MISO
     - Master Input, Slave Output (data from peripheral to autopilot)
     - MISO
     - MISO
     - Black
     - Blue
   * - MOSI
     - Master Output, Slave Input (data from autopilot to peripheral)
     - MOSI
     - MOSI
     - Black
     - Green
   * - CS
     - Chip Select, activates the specific peripheral
     - CS
     - CS
     - Black
     - White
   * - GND
     - Ground reference
     - GND
     - GND
     - Black
     - Black

.. note:: SPI signal names can be labeled in many different ways. See the `SPI Wikipedia entry <https://en.wikipedia.org/wiki/Serial_Peripheral_Interface>`__ for more information.

Cable Recommendations
+++++++++++++++++++++

SPI signals can be sensitive to electromagnetic interference (EMI) and crosstalk, especially at higher clock rates. To minimize these issues:

- Keep cable lengths as short as possible.
- Ensure signal cables are separated from high-power and noisy cables.

SAFETY SW/LED
-------------

Many autopilots provide dedicated GPIOs on a connector for adding the optional safety switch and notification leds that ArduPilot support. Usually these are offered on autopilots that utilize an IOMCU co-processor.

Signals
+++++++

- +3.3V :  Supply to the LED and Switch
- LED:     Drives the ground side of the notification LED
- SW:      Senses if +3.3V is present to indicate switch closure

BUZZER
------

A -BUZZ output is sometimes provided for a passive or active buzzer for system notification sounds, and provides a switched ground connection to the buzzer. See :ref:`Buzzer<common-buzzer>` 

ANALOG INPUTS
-------------

Often analog voltage measurement pins are provided. These are used for current and/or voltage sensing from a power monitor (if a dedicated connector has not been provided), other system voltage monitor points, or for analog :ref:`RSSI<common-rssi-received-signal-strength-indication>` input.

Cable Colour Coding
-------------------

A clear and consistent cable colour-coding system is essential for quick identification and correct assembly of drone cables. Although different manufacturers may use slightly varying schemes, adhering to a standardized colour-coding method helps significantly in identifying cables quickly, reducing wiring mistakes, and simplifying maintenance or troubleshooting procedures. This practice is especially beneficial when referencing photographs in manuals or documentation, as it helps users visually distinguish between different cable types and their intended use.

Recommended Cable Colour Coding
+++++++++++++++++++++++++

The following table illustrates a recommended cable colour coding based on commonly used conventions:

.. list-table:: Recommended Cable Colour Coding
   :header-rows: 1
   :widths: 15 55

   * - Color
     - Preferred Usage
   * - Red
     - Power voltage (+5V, +12V, main power)
   * - Black
     - Ground, power return ground
   * - Green
     - General-purpose signals, data lines
   * - White
     - General-purpose signals, data lines
   * - Yellow
     - General-purpose signals, data lines
   * - Blue
     - Power return, open-collector control signals

General Rules
+++++++++++++

To enhance clarity and avoid mistakes, adhere to the following rules when designing cable harnesses:

- Reserve red and black strictly for power and ground lines, respectively.
- Use the same colour consistently for the same type of signal throughout the harness.
- Avoid repeating the same colour for adjacent wires in a connector.
- Ensure wiring harnesses with the same pin count have a unique colour sequence to clearly identify the cable type.

Using a clear colour-coding scheme greatly simplifies documentation processes. Photographs used in manuals or assembly instructions become clearer and easier to understand when cable colours distinctly identify their function. 

The above recommendations are adopted by ThunderFly s.r.o and reflect common industry best practices. For more details, refer to the `Pixhawk Connector Standard <https://github.com/PX4/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf>`_.


