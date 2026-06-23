.. _common-external-ahrs-xsens:

===================
Xsens External AHRS
===================

`Xsens <https://www.movella.com/products/sensor-modules>`__ (Movella) MTi-series
IMU / AHRS / GNSS-INS modules can be used as an External AHRS in ArduPilot. The
Xsens module runs its own onboard sensor fusion and provides attitude, position,
and velocity to ArduPilot, replacing the internally generated INS/AHRS
subsystems.

Supported Devices
=================

The driver communicates with the Xsens module using the Xbus protocol and
supports the following GNSS/INS-capable products:

=============  ====================  ============  ===============  ================
Device         Type                  GNSS          Interface        Series
=============  ====================  ============  ===============  ================
MTi-7          AHRS / GNSS-INS       External      UART / SPI       MTi 1-series
MTi-8          AHRS / RTK GNSS-INS   External RTK  UART / SPI       MTi 1-series
MTi-670(G)     GNSS/INS              Internal      UART (RS232)     MTi 600-series
MTi-680(G)     RTK GNSS/INS          Internal RTK  UART (RS232)     MTi 600-series
MTi-G-710      GNSS/INS              Internal      UART             MTi 100-series
=============  ====================  ============  ===============  ================

The MTi 1-series modules (MTi-7 / MTi-8) support both UART and SPI interfaces.

Connecting the Sensor
=====================

.. note::

   The MTi 1-series modules (MTi-7 / MTi-8) require a **3.3V** supply. A 5V
   supply will power the LED but the data link will not work. Power is
   independent from the data interface; selecting UART or SPI does not change
   how the module is powered.

UART
----

Connect the module to a serial port on the autopilot:

=================  ===  =============
Autopilot          ->   Xsens module
=================  ===  =============
TX                 ->   RX
RX                 ->   TX
GND                ->   GND
=================  ===  =============

On the MTi 1-series, set the interface-select pins for UART:
``PSEL0 = GND``, ``PSEL1 = GND``.

The MTi 600-series (MTi-670/680) and MTi-G-710 use RS232 signal levels by
default and require an RS232-to-3.3V-TTL level converter between the module and
the autopilot serial port.

SPI (MTi 1-series)
------------------

Set the interface-select pins for SPI: ``PSEL0 = GND``, ``PSEL1 = float``.

=================  =============
Signal             Xsens module
=================  =============
NCS (chip select)  SPI CS
SCLK               SPI clock
MOSI               data in
MISO               data out
GND                GND
=================  =============

Ensure the module, its power supply, and the autopilot share a common ground.

.. note::

   The Data Ready (DRDY) line is optional. When the board definition does not
   provide a DRDY GPIO, the driver operates in polling mode.

Configuration
=============

Set the following parameters and reboot the autopilot.

:ref:`AHRS_EKF_TYPE<AHRS_EKF_TYPE>` = 11 (External AHRS)

:ref:`EAHRS_TYPE<EAHRS_TYPE>` = 12 (Xsens)

For **UART** connection, also set, on the serial port used:

- :ref:`SERIALx_PROTOCOL<SERIAL1_PROTOCOL>` = 36 (AHRS)
- :ref:`SERIALx_BAUD<SERIAL1_BAUD>` = 115

For **SPI** connection (MTi 1-series), set:

- :ref:`EAHRS_OPTIONS<EAHRS_OPTIONS>` = 16

The ``EAHRS_OPTIONS`` bits used by the Xsens driver are:

============  =====  ===================================
Bit           Value  Meaning
============  =====  ===================================
3             8      Sensor mounted facing downward
4             16     Use SPI instead of UART
============  =====  ===================================

For example, a downward-mounted sensor on SPI is ``EAHRS_OPTIONS`` = 24.

.. note::

   The External AHRS parameters may not be visible until ``SERIALx_PROTOCOL``
   is set and the parameters are refreshed, or the autopilot is rebooted.

Verification
============

After rebooting, the Mission Planner Messages tab should show the External AHRS
initialising and the IMU and barometer being registered as external. Moving the
sensor should cause the attitude shown on the HUD to follow in real time.

Notes
=====

- The Xsens module performs sensor fusion onboard and provides the attitude
  solution directly. Setting ``AHRS_EKF_TYPE`` = 11 configures ArduPilot to use
  this external solution.
- GNSS-enabled products additionally provide position and velocity to ArduPilot.
