.. _common-ads-b-receiver:


==============
ADS-B
==============

This article describes how to attach and configure an ADS-B module so that your aircraft can be aware of, and/or transmit to, other aircraft and air-traffic control nearby. This also allows the pilot on the ground to be aware of nearby manned aircraft and optionally to allow the vehicle to automatically avoid them.

   ..  youtube:: boe-25OI4bM
    :width: 100%

ADS-B (aka `Automatic Dependent Surveillance Broadcast <https://en.wikipedia.org/wiki/Automatic_dependent_surveillance_%E2%80%93_broadcast>`__) is an air traffic surveillance technology that enables aircraft to be accurately tracked by air traffic controllers and other pilots without the need for conventional radar.

.. note::

   uAvionix ADS-B Ping support was introduced in Plane-3.5 and Copter-3.4. Simple avoidance was added to Plane-3.5 and a more advanced avoidance was added to Plane-3.7 and Copter-3.4.

.. warning::

   The avoidance features are new features still under development and should be used with caution. It is highly recommended that the RCx_OPTION =38 (ADSB Avoidance En) feature be setup if ADSB avoidance is enabled to allow easy disabling while airborne, if so desired.

Required Hardware
=================

The uAvionix Ping sensor can be purchased directly from `uAvionix <https://uavionix.com/products/>`__ or from the following vendors:

   -  USA: `Unmanned Systems Source <https://www.unmannedsystemssource.com/shop/atc-devices/pingrx-ads-b-receiver/>`__
   -       `R Cubed Engineering <http://www.rcubedengineering.com/ecommerce/>`__
   -  U.K.: `Unmanned Tech <http://www.unmannedtech.co.uk/>`__
   -  Germany: `UAV Store <http://www.uav-store.de/ads-b-receivers/>`__
   -  Asia: `jDrones pingRX <http://store.jdrones.com/ping_ads_b_receiver_p/adsbping01.htm>`__

The full reseller list can be found at `uAvionix <https://uavionix.com/resellers/>`__


Connecting to the autopilot
===================================

.. image:: ../../../images/adsb_and_pixhawk.png
    :target: ../_images/adsb_and_pixhawk.png

The ADSB receiver comes with a DF13 serial cable that can be plugged
directly into a Pixhawk serial port.
The Ping sensor should be mounted so that the antenna is oriented
vertically.

Setup through the ground station
================================

Set the :ref:`ADSB_TYPE <ADSB_TYPE>` parameter to "1" to enable receiving data from the Uavonix ADSB sensor.

If you are using one of the UARTs on your board which defaults to MAVLink (i.e. Telem1, Telem2 on a Pixhawk) then the default settings will work fine for the PingRx. Alternatively you
can connect the Ping to one of the other UARTs, such as the GPS UART (if it is unused) or the serial4/5 UART. In that case you will need to configure the UART as MAVLink at a baudrate of 57600.

For example, if you plugged the Ping into "serial4/5" on a
Pixhawk you would set:

-  :ref:`SERIAL4_PROTOCOL <SERIAL4_PROTOCOL>` to 1 (meaning MAVLink)
-  :ref:`SERIAL4_BAUD <SERIAL4_BAUD>` 57 (meaning 57600)


For the Ping2020 you'll need to set the _PROTOCOL value to 2. For example, when connected to Telem2 you would set:

-  :ref:`SERIAL2_PROTOCOL <SERIAL4_PROTOCOL>` to 2 (meaning MAVLink v2.0)

You will need to reboot your board after making those changes.

To enable streaming the ADSB data to the GCS you'll want to check your StreamRate param. In some cases it is already set but it's good to check. These rates are adjustable per telemetry like in the case of having both a high-bandwidth and a low-bandwitdh link attached. The param to adjust the rate would depend on which one your GCS is connected to. In most cases, it is telem1.

-  :ref:`SR1_ADSB <SR1_ADSB>` 5 (meaning 5Hz)

Once operational aircraft within about 50km should appear on the ground
station map.

.. image:: ../../../images/ADSB_MissionPlanner.jpg
    :target: ../_images/ADSB_MissionPlanner.jpg

To test the system you can compare with flights shown on
`flightradar24.com <https://www.flightradar24.com/>`__.

ADSB-out configuration
======================================

.. warning::

   Ensure you have the correct permissions to be using ADSB hardware that is capable of transmitting. You will be showing up on air-traffic controller airport radar!
   
The following parameters are used to configure ADS-B out:

-  :ref:`ADSB_ICAO_ID <ADSB_ICAO_ID>` : ICAO_ID unique vehicle identification number of this aircraft. This is a integer limited to 24bits. If set to 0 then one will be randomly generated. If set to -1 then static information is not sent, transceiver is assumed pre-programmed.
-  :ref:`ADSB_EMIT_TYPE <ADSB_EMIT_TYPE>` : ADSB classification for the type of vehicle emitting the transponder signal. Default value is 14 (UAV).
-  :ref:`ADSB_LEN_WIDTH <ADSB_LEN_WIDTH>` : Aircraft length and width dimension options in Length and Width in meters. In most cases, use a value of 1 for smallest size.
-  :ref:`ADSB_OFFSET_LAT <ADSB_OFFSET_LAT>` : GPS antenna lateral offset. This describes the physical location offest from center of the GPS antenna on the aircraft.
-  :ref:`ADSB_OFFSET_LON <ADSB_OFFSET_LON>` : GPS antenna longitudinal offset. This is usually set to 1, Applied By Sensor
-  :ref:`ADSB_RF_SELECT <ADSB_RF_SELECT>` : Transceiver RF selection for Rx enable and/or Tx enable. This only effects devices that can Tx and/or Rx. Rx-only devices override this to always be Rx-only.
-  :ref:`ADSB_SQUAWK <ADSB_SQUAWK>` : Squawk/Transponder (Mode 3/A) code that is braodcasted to ATC that is usually assigned by your ATC for a given flight. In the USA/Canada the default squawk code is for VFR which is 1200. Most parts of Europe and Australia use 7000. If an invalid octal number is set then it will be reset to 1200.

In many cases the defaults are OK and you don't need to change any of these except `ADSB_RF_SELECT <ADSB_RF_SELECT>` which is needed to turn on the transmitter. The ADSB_RF_SELECT transmit bit is cleared on boot to ensure you're only trsnamitting when intentionally enabled.
There are additional MAVLink messages for ADSB in uavionix.xml to allow a GCS to set all of these options. Namely, msg UAVIONIX_ADSB_OUT_CFG and UAVIONIX_ADSB_OUT_DYNAMIC where the _cfg is the only place where you can assign a custom callsign.

Enabling Manned Vehicle Avoidance
=================================

ArduPilot includes a flight mode, AVOID_ADSB, that attempts to avoid manned vehicles based on the ADS-B sensor's output. Entry into this mode is automatic when avoidance is necessary based on the parameters below. Exit is also automatic when the threat has passed.

To enable this feature connect with a Ground Station and set the following parameters:

-  :ref:`AVD_ENABLE <AVD_ENABLE>` : set to "1" to enable ADS-B based avoidance (param refresh may be necessary after setting this)
-  :ref:`AVD_F_DIST_XY <AVD_F_DIST_XY>` : the horizontal distance in meters that should be considered a near-miss
-  :ref:`AVD_F_DIST_Z <AVD_F_DIST_Z>` : the vertical distance in meters above or below the vehicle that should be considered a near-miss
-  :ref:`AVD_F_TIME <AVD_F_TIME>` : how many seconds in advance of a projected near-miss (based on the vehicle's current position and velocity) the vehicle should begin the ``AVD_F_ACTION``.
-  :ref:`AVD_F_ACTION <AVD_F_ACTION>` : controls how the vehicle should respond to a projected near-miss (i.e. 2:Climb Or Descend, 3:Move Horizontally, 4:Move Perpendicularly in 3D, 5:RTL or 6:Hover)
-  :ref:`AVD_F_RCVRY <AVD_F_RCVRY>` : sets how the vehicle will behave after the vehicle has cleared the near-miss area (i.e. 1 = resume previous flight mode)

Note: there are equivalent "Warn" parameters (i.e. AVD_W_DIST_XY) that can be used to adjust when warnings to the pilot will appear on the ground station.

In ArduPilot firmware versions 4.0 and later, the entry into this mode can be enabled or disabled via an RC channel switch by setting the channel's RCx_OPTION = 38 (ADSB Avoidance En). If the RC PWM is >1800us, then entry into this mode is enabled if a threat presents.

.. warning::

   The avoidance features are still under development and should be used with caution.  They may not yet be useful for real-life manned vehicle avoidance.

   ..  youtube:: quomxCIPP74
    :width: 100%

Vehicle Database
================

When enabled, the ADS-B library will store information for up to 50 vehicles
detected by the ADS-B receiver but can be further limited using the
``ADSB_LIST_SIZE`` parameter. Due to some experimental work
in other features, such as EKF2, available RAM may be limited. It is
important to note that when ADS-B is disabled (ADSB_ENABLE=0) then the
memory is released, effectively freeing up about 1KB of RAM. When
enabled, the detected vehicle list is checked once per second for
potential conflicts.

Developer information including Simulation
==========================================
The data is transmitted via the `ADSB_VEHICLE message <https://mavlink.io/en/messages/common.html#ADSB_VEHICLE>`__. When
received by ArduPilot, it is streamed out using the SRx_ADSB value where x is the telemetry port number and the
value is how many vehicles per second to be streamed. If using telem1 the streamrate param would be ``SR1_ADSB``. The list will not repeat any faster than 1 second. This
flexibility is useful to conserve bandwidth on data links but also allow maximum update rate for high-speed links
such as an on-board companion computer.

ArduPilot's SITL includes the simulation of ADS-B enabled aircraft.
To enable this you must have pymavlink v1.1.70 or greater. If you have
an older version, use:

::

    sudo pip install --upgrade pymavlink MAVProxy

Set the number of aircraft to simulate using the ``SIM_ADSB_COUNT`` parameter. Ping2020 simulation support
can be enabled by setting parameter ``SIM_ADSB_TX``. Other simulation options for ADS-B are present, all
starting with ``SIM_ADSB_``.

Plugging in a hardware ADS-B receiver to your computer using a USB-to-Serial converter, or using the PingUSB, will allow you to overlay real ADS-B
traffic into the simulation.  You might invoke SITL in this way to achieve this effect:

::

   sim_vehicle.py -v ArduCopter -A "--uartC uart:$SERIAL_DEVICE:57600"

Where SERIAL_DEVICE might be /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A4008ZND-if00-port0 on a Linux system (find a list of valid serial devices with the command ``ls /dev/serial/by-id/*`` or ``ls /dev/ttyS*`` for a COM port on Cygwin).  Once SITL has started it may be necessary to set the ``SERIAL3_`` parameters:

::

   SERIAL3_PROTOCOL 1
   SERIAL3_BAUD 57600

