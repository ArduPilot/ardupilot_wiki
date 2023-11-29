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

By default (for most autopilots) the protocols/expected peripheral for each port is shown below:

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

MAVLink1, see :ref:`Telemetry Setup<common-telemetry-port-setup>`

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>2</td>
   <td>

MAVLink2, see :ref:`Telemetry Setup<common-telemetry-port-setup>`

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>3</td>
   <td>

FrSky D, see :ref:`FrSky Telemetry <common-frsky-telemetry>` 

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>4</td>
   <td>

FrSky SPort, see :ref:`FrSky Telemetry <common-frsky-telemetry>` 

.. raw:: html

   </td>
   <tr>
   <td>5</td>
   <td>

GPS, see :ref:`GPS <common-gps-how-it-works>` and :ref:`Devices<common-positioning-landing-page>`

.. raw:: html

   </td>
   </tr>
      <tr>
   <td>7</td>
   <td>

Alexmos Gimbal Serial, see :ref:`Alexmos Gimbal <common-simplebgc-gimbal>` 

.. raw:: html

   </td>
   </tr>
      <tr>
   <td>8</td>
   <td>

SToRM32 Gimbal Serial, see :ref:`SToRM32 Gimbal <common-storm32-gimbal>` 

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>9</td>
   <td>

Rangefinder, see :ref:`Rangefinders <common-rangefinder-landingpage>` 

.. raw:: html

   </td>
   </tr>
      <tr>
   <td>10</td>
   <td>

FrSky SPort Passthrough (OpenTX), see :ref:`FrSky Passthrough Telemetry <common-frsky-passthrough>` 

.. raw:: html

   </td>
   </tr>
      <tr>
   <td>11</td>
   <td>

Lidar360, see :ref:`360 Lidars here <common-rangefinder-landingpage>` 

.. raw:: html

   </td>
   </tr>
      <tr>
   <td>13</td>
   <td>

Beacon, see :ref:`Non-GPS Navigation <common-non-gps-navigation-landing-page>` 

.. raw:: html

   </td>
   </tr>
      <tr>
   <td>14</td>
   <td>

Volz Servo, see :ref:`common-servo-volz` 

.. raw:: html

   </td>
   </tr>
      <tr>
   <td>15</td>
   <td>

SBus Servo, see :ref:`common-sbus-output` 

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>16</td>
   <td>

ESC Telemetry, see :ref:`blheli32-esc-telemetry` 

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

OpticalFlow, see :ref:`Optical Flow Sensors <common-optical-flow-sensors-landingpage>` 

.. raw:: html

   </td>
   </tr>
      <tr>
   <td>19</td>
   <td>

RobotisServo, see :ref:`common-servo-robotis` 

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

WindVane, see :ref:`wind-vane` 

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

RC Input, see :ref:`common-flight-controller-wiring` 

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
   <tr>
   <td>26</td>
   <td>

Runcam see :ref:`common-camera-runcam` 

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>27</td>
   <td>

HOTT Telem see :ref:`common-hott-telemetry`

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>28</td>
   <td>

Scripting see :ref:`common-lua-scripts` 

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>29</td>
   <td>

Crossfire Receiver :ref:`common-tbs-rc`

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>30</td>
   <td>

Generator see :ref:`common-richenpower-generator`

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>31</td>
   <td>

Winch

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>32</td>
   <td>

MSP Telemetry see :ref:`common-msp-overview`

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>33</td>
   <td>

DJI FPV telemetry see :ref:`common-msp-osd-overview`

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>34</td>
   <td>

Serial Airspeed sensor

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>35</td>
   <td>

Serial ADSB receiver

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>36</td>
   <td>

External AHRS, see :ref:`common-external-ahrs`

.. raw:: html

   </td>
   </tr>
    <tr>
   <td>37</td>
   <td>

Smart Audio, see :ref:`common-vtx`

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>38</td>
   <td>

FETtecOneWire, see :ref:`common-fettec-onewire`

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>39</td>
   <td>

Torqeedo, see :ref:`common-torqeedo`

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>40</td>
   <td>

AIS, see :ref:`common-ais`

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>41</td>
   <td>

CoDevESC

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>42</td>
   <td>

DisplayPort, see :ref:`common-msp-osd-overview-4.2`

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>43</td>
   <td>

MAVLink High Latency, see :ref:`common-MAVLink-high-latency`

.. raw:: html

   </td>
   </tr>
   <tr>
   <td>44</td>
   <td>

IRC Tramp, see :ref:`common-vtx`

.. raw:: html

   </td>
   </tr>
   <tr>
   <td> 45</td>
   <td>

DDS XRCE

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
- if bit 4 is set, then the RX pin has a weak pull down resistor activated.
- if bit 5 is set, then the RX pin has a weak pull up resistor activated.
- if bit 6 is set, then the TX pin has a weak pull down resistor activated.
- if bit 7 is set, then the TX pin has a weak pull up resistor activated.
- if bit 8 is set, then the RX has no DMA activated (assuming DMA is available on this UART)
- if bit 9 is set, then the TX has no DMA activated (assuming DMA is available on this UART)
- if bit 10 is set, then MAVLink forwarding will not be active on this UART port.
- if bit 11 is set, then the hardware FIFO in H7 autopilots is disabled
- if bit 12 is set, the GCS are prevented from changing the MAVLink message stream rates set by the ``SRx_...`` parameters.

.. note:: HalfDuplex is supported on all ChiBiOS based autopilots, but inversion and swap are only supported on boards with F7 or H7 microprocessors.
