.. _common-cabling-guide:

========================
Cable Design Guidelines
========================

This guide provides detailed recommendations for cabling with a focus on signal integrity, electromagnetic compatibility, wire color conventions, connector standards, and best practices for specific signals like I2C and CAN. These practices help ensure reliable operation, simplify debugging, and improve documentation clarity across unmanned platforms of all sizes.

Cable Colour Coding
-------------------

A consistent cable colour-coding system is useful for correct assembly and maintenance. It also improves clarity in documentation and field repairs.

.. list-table:: Recommended Cable Colour Coding
   :header-rows: 1
   :widths: 15 55

   * - Colour
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

General Rules:

- Red and black are reserved for power and ground.
- Use consistent colours per signal type across harnesses.
- Avoid repeating colours for adjacent wires in a connector.
- Cables with the same pin count should have unique colour sequences — this improves identification in manuals and photos.

I2C
---

The I2C bus is widely used by autopilots to connect low-bandwidth peripheral components such as magnetometers (compasses), digital airspeed sensors, rangefinders, tachometers, and other sensors. Multiple devices can be connected to a single I2C bus, each identified by a unique address. I2C employs a two-wire open-drain communication method using Serial Clock (SCL) and Serial Data (SDA) lines. The bus lines are typically pulled up to a positive voltage (3.3V or 5V) through resistors, ensuring a default idle state at logic high.

Signals
+++++++

.. list-table:: I2C Signal Colour Examples
   :header-rows: 1
   :widths: 15 20 20 20

   * - Signal
     - Pixhawk
     - ThunderFly
     - CUAV
   * - +5V
     - Red
     - Red
     - Red
   * - SCL
     - Black
     - Yellow
     - White
   * - SDA
     - Black
     - Green
     - Yellow
   * - GND
     - Black
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
  - `Using address translator <https://docs.thunderfly.cz/avionics/TFI2CADT01/>`_, which allows multiple devices with identical addresses to coexist by remapping their addresses.

- **Excessive Wiring Capacitance**: Long cables or multiple devices connected to a single bus significantly increase cable capacitance, resulting in degraded signal quality. Possible solutions include:
  - Reducing cable length or using higher-quality cables.
  - Splitting devices across multiple I2C buses.
  - `Employing I2C bus accelerators or extenders <https://docs.thunderfly.cz/avionics/TFI2CEXT01/>`_, to boost signal quality over extended cable lengths.

I2C Bus Accelerators and Translators
++++++++++++++++++++++++++++++++++++

- **I2C Bus Accelerators** (e.g., `ThunderFly TFI2CEXT01 <https://docs.thunderfly.cz/avionics/TFI2CEXT01/>`_) physically divide the bus into segments, amplifying signals and reducing the impact of wiring capacitance, thus improving signal integrity and reliability on longer cables. The TFI2CEXT01 can also perform voltage level translation between 3.3V and 5V logic levels.
- **I2C Address Translators** (e.g., `ThunderFly TFI2CADT01 <https://docs.thunderfly.cz/avionics/TFI2CADT01/>`_) allow multiple identical devices with the same I2C address to be connected by remapping device addresses dynamically, resolving conflicts and simplifying sensor integration.

CAN
---

:ref:`CAN <common-canbus-setup-advanced>` bus is provided on many autopilots for use with :ref:`DroneCAN <common-uavcan-setup-advanced>` peripherals. ESCs, GPS/Compass, Rangefinders, and many other peripherals and sensors are being added to the list of available DroneCAN devices everyday. CAN provides a robust method of communicating with peripherals with data integrity, even with long leads.


Signals
+++++++

Power and ground lines are typically provided alongside the CAN signals on standard 4-pin connectors (e.g., JST-GH).

.. list-table:: CAN Signal Colour Examples
   :header-rows: 1
   :widths: 15 20 20 20 20

   * - Signal
     - Pixhawk
     - ThunderFly
     - CUAV
     - Zubax
   * - +5V
     - Red
     - Red
     - Red
     - Red
   * - CAN_H
     - Black
     - White
     - White
     - White
   * - CAN_L
     - Black
     - Yellow
     - Yellow
     - Yellow
   * - GND
     - Black
     - Black
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

.. list-table:: SPI Signal Colour Examples
   :header-rows: 1
   :widths: 15 20 20

   * - Signal
     - Pixhawk
     - ThunderFly
   * - +5V
     - Red
     - Red
   * - SCK
     - Black
     - Yellow
   * - MISO
     - Black
     - Blue
   * - MOSI
     - Black
     - Green
   * - !CS1
     - Black
     - White
   * - !CS2
     - Black
     - Blue
   * - GND
     - Black
     - Black


.. note:: SPI signal names can be labeled in many different ways. See the `SPI Wikipedia entry <https://en.wikipedia.org/wiki/Serial_Peripheral_Interface>`__ for more information.

Cable Recommendations
+++++++++++++++++++++

SPI signals can be sensitive to electromagnetic interference (EMI) and crosstalk, especially at higher clock rates. To minimize these issues:

- Keep cable lengths as short as possible.
- Ensure signal cables are separated from high-power and noisy cables.

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

