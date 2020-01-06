.. _common-serial-options:

=================================
Serial Port Configuration Options
=================================

This page describes the configuration options for the serial ports. Currently, some of these options are supported only on specific autopilots.

Logical Serial Port to Physical UART Assignment
===============================================

ArduPilot Serialx Port numbering is logical, rather than physical. Which UART or USART port is assigned to a Serial Port is determined by the autopilot's hardware definition file. 
Serial Port 0 is always assigned to the USB port, but others can vary. Check its  :ref:`description page <common-autopilots>`

.. note:: more serial ports may be shown in the parameters than exist on a given controller. Check its  :ref:`description page<common-autopilots>` 

By default the protocols/expected peripheral for each port is shown below:

.. note:: any supported protocol/peripheral can be used by any port by changing its ``SERIALx_PROTOCOL`` parameter

+-----------------+------------------------------------+
|Serial 0         | USB port, MAVLink2 protocol        |
+-----------------+------------------------------------+
|Serial 1         | Telemetry port 1, MAVLink1 protocol|
+-----------------+------------------------------------+
|Serial 2         | Telemetry port 2, MAVLink1 protocol|
+-----------------+------------------------------------+
|Serial 3         | GPS1 port                          |
+-----------------+------------------------------------+
|Serial 4         | GPS2 port                          |
+-----------------+------------------------------------+
|Serial 5         | USER port, disabled                |
+-----------------+------------------------------------+
|Serial 6         | USER port, disabled                |
+-----------------+------------------------------------+
|Serial 7         | USER port, disabled                |
+-----------------+------------------------------------+

Often cased autopilots will have the designation "TELEM1", "GPS", etc. marked on the case, otherwise, the autopilot :ref:`description page<common-autopilots>`  should provide the mapping to SERIALx Port

SERIALx_PROTOCOL
================

The serial ports support many different kinds of interfaces and peripheral. The ``SERIALx_PROTOCOL`` parameter determines what type of device/interface is expected.

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Value</th>
   <th>Protocol</th>
   </tr>
   <tr>
   <td>-1</td>
   <td>Disabled</td>
   </tr>
   <tr>
   <td>1</td>
   <td>

MAVLink1, See :ref:`Telemetry Setup<common-telemetry-port-setup>`

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>2</td>
   <td>

MAVLink2, See :ref:`Telemetry Setup<common-telemetry-port-setup>`

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>3</td>
   <td>

FrSky D, See :ref:`FrSky Telemetry <common-frsky-telemetry>` 

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>4</td>
   <td>

FrSky SPort, See :ref:`FrSky Telemetry <common-frsky-telemetry>` 

.. raw:: html

   </td>
   <tr>
   <td>5</td>
   <td>

GPS, See :ref:`GPS <common-gps-how-it-works>` and :ref:`Devices<common-positioning-landing-page>`

.. raw:: html

   </td>
   </tr>
      <tr>
   <td>7</td>
   <td>

Alexmos Gimbal Serial, See :ref:`Alexmos Gimbal <common-simplebgc-gimbal>` 

.. raw:: html

   </td>
   </tr>
      <tr>
   <td>8</td>
   <td>

SToRM32 Gimbal Serial, See :ref:`SToRM32 Gimbal <common-storm32-gimbal>` 

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>9</td>
   <td>

Rangefinder, See :ref:`Rangefinders <common-rangefinder-landingpage>` 

.. raw:: html

   </td>
   </tr>
      <tr>
   <td>10</td>
   <td>

FrSky SPort Passthrough (OpenTX), See :ref:`FrSky Passthrough Telemetry <common-frsky-passthrough>` 

.. raw:: html

   </td>
   </tr>
      <tr>
   <td>11</td>
   <td>

Lidar360, See :ref:`360 Lidars here <common-rangefinder-landingpage>` 

.. raw:: html

   </td>
   </tr>
      <tr>
   <td>13</td>
   <td>

Beacon, See :ref:`Non-GPS Navigation <common-non-gps-navigation-landing-page>` 

.. raw:: html

   </td>
   </tr>
      <tr>
   <td>14</td>
   <td>

Volz Servo, See :ref:`common-servo-volz` 

.. raw:: html

   </td>
   </tr>
      <tr>
   <td>15</td>
   <td>

SBus Servo, See :ref:`common-sbus-output` 

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>16</td>
   <td>

ESC Telemetry, See :ref:`common-dshot` 

.. raw:: html

   </td>
   </tr>
      <tr>
   <td>17</td>
   <td>

Devo Telemetry

.. raw:: html

   </td>
   </tr>
      <tr>
   <td>18</td>
   <td>

OpticalFlow, See :ref:`Optical Flow Sensors <common-optical-flow-sensors-landingpage>` 

.. raw:: html

   </td>
   </tr>
      <tr>
   <td>19</td>
   <td>

RobotisServo, See :ref:`common-servo-robotis` 

.. raw:: html

   </td>
   </tr>
      <tr>
   <td>20</td>
   <td>
	NMEA Output, NEMA Output stream from GPS

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>21</td>
   <td>

WindVane, See :ref:`wind-vane` 

.. raw:: html

   </td>
   </tr>
      <tr>
   <td>22</td>
   <td>

SLCAN

.. raw:: html

   </td>
   </tr>
      <tr>
   <td>23</td>
   <td>

RC Input, See :ref:`common-flight-controller-wiring` 

.. raw:: html

   </td>
   </tr>
      <tr>
   <td>24</td>
   <td>

MegaSquirt EFI, see `MegaSquirt EFI <http://megasquirt.info/>`__

.. raw:: html

   </td>
   </tr>
      <tr>
   <td>25</td>
   <td>

LTM Telemetry, see :ref:`LTM Telemetry <common-ltm-telemetry>` 

.. raw:: html

   </td>
   </tr>
   </tbody>
   </table>


SERIALx_OPTIONS Parameter
=========================

Every serial port has in addition, to its baud rate (``SERIALx_BAUD``) and protocol format (``SERIALx_PROTOCOL``), the ability to invert its RX input and/or TX data, operate in half-duplex mode, and/or swap its RX and TX inputs.

For example, for direct connection to FRSky SPort telemetry, normally inverters and diode OR externally would be required. With SERIALx_OPTIONS bitmask set to 7, direct connection to the SPort can be accomplished from a serial port.

Bitmask Options
---------------

- if bit 0 is set, then RX data received is inverted internally.
- if bit 1 is set, the TX data is inverted before outputting.
- if bit 2 is set, then HalfDuplex operation using the TX pin is implemented.
- if bit 3 is set, then the TX and RX pins are effectively swapped internally.

.. note:: HalfDuplex is supported on all ChiBiOS based autopilots, but all other options are only supported on boards with F7 or H7 microprocessors.