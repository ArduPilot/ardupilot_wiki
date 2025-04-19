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
  - Changing the device's address via hardware or software configuration, if supported.
  - Use an address translator to remap address, such as from `ThunderFly <https://docs.thunderfly.cz/avionics/TFI2CADT01/>`__ or `Adafruit <https://www.adafruit.com/product/5914>`__.

- **Excessive Wiring Capacitance**: Long cables or multiple devices connected to a single bus significantly increase cable capacitance, resulting in degraded signal quality. Possible solutions include:

  - Reducing cable length or using higher-quality cables.
  - Splitting devices across multiple I2C buses.
  - Use an I2C bus extender such as `Adafruit I2C Extender <https://www.adafruit.com/product/4756>`__ , or `ThundeFly's I2C bus accelerators or extenders <https://docs.thunderfly.cz/avionics/TFI2CEXT01/>`__ to boost signal quality over extended cable lengths.

CAN
---
:ref:`CAN <common-canbus-setup-advanced>` bus is provided on many autopilots for use with :ref:`DroneCAN <common-uavcan-setup-advanced>` peripherals. ESCs, GPS/Compass, Rangefinders, and many other peripherals and sensors are being added to the list of available DroneCAN devices everyday. CAN provides a robust method of communicating with peripherals with data integrity, even with long leads.

Cable Recommendations
+++++++++++++++++++++

CAN cables should use twisted pairs to reduce electromagnetic interference (EMI) and maintain signal integrity on cables over 6 inches:

- Twist each signal pair (CAN_H/CAN_L and +5V/GND) approximately 10 turns per 30 cm.
- Twist both pairs together about 4 turns per 30 cm.
- Maintain separation from high-power and high-noise cables.

Due to its differential signaling and robust protocol, CAN is particularly suitable for applications requiring reliability over longer cable lengths and in electrically noisy environments.

SPI
---
SPI (Serial Peripheral Interface) is a synchronous serial communication protocol used by autopilots to connect higher-bandwidth peripherals and sensors, such as optical flow sensors, specialized telemetry modems, barometers, IMUs, and other advanced digital sensors. It supports full-duplex communication using separate lines for data input and output. Most autopilots have processors with multiple SPI ports.

Cable Recommendations
+++++++++++++++++++++
SPI signals can be sensitive to electromagnetic interference (EMI) and crosstalk, especially at higher clock rates. To minimize these issues:

- Keep cable lengths as short as possible.
- Ensure signal cables are separated from high-power and noisy cables.

UART
----
UART (Universal Asynchronous Receiver/Transmitter) ports are commonly provided by autopilots to connect peripherals such as telemetry radios, GPS receivers, rangefinders, radio modems, and even SBUS servos. Serial RC receiver inputs can be connected to any UART. See :ref:`Serial Port Configuration <common-serial-options>`. Usually, several UART ports are provided. 

By default, UART connections are point-to-point and do not support bus-style networking. Each UART line directly connects two devices. The connecting cable is typically straight (1:1), meaning no crossing is performed within the cable itself. Therefore, the peripheral devices must internally cross RX/TX signals as required.

Some UART ports provide CTS (Clear To Send) and RTS (Request To Send) signals to allow  a hardware handshake mechanism that improves reliability by preventing data overflow. If the peripheral does not utilize these signals, they can typically remain disconnected. ArduPilot also provides a means to disable this flow control using the ``BRD_SERx_RTSCTS`` parameters.

Cable Recommendations
+++++++++++++++++++++
UART signals, particularly at higher baud rates, can generate electromagnetic interference (EMI). However, this is not normally an issue in most vehicles. However, to minimize EMI:

- Keep UART cable lengths as short as practical.
- Separate UART cables from high-power wires and sensitive sensors, such as GPS receivers and magnetometers.

Unlike twisted-pair cables used for differential protocols (CAN, I2C), cable twisting is generally not applicable for UART signals due to their single-ended nature.

