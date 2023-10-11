.. _common-mlrs-rc:
[copywiki destination="plane,copter,rover,blimp"]
============
mLRS project
============

`mLRS firmware github page <https://github.com/olliw42/mLRS>`__

`mLRS documentation <https://github.com/olliw42/mLRS-docu/blob/master/README.md>`__

The mLRS project is open source firmware which implements a long range integrated RC and full MAVLink telemetry system which can run on a few off the shelf and DIY hardware platforms.  It supports 433 MHz, 868/915 MHz, and 2.4 GHz bands.  It is capable of reaching ranges of 7 to 87 km in LoRa modes, depending on the message rate, output power, and frequency band supported by the chosen hardware.  A MAVLink ground station (phone or PC) may be connected to the transmitter module via USB, serial, WiFi or Bluetooth depending on the hardware selected.

When the CRSF protocol is used for RC data on EdgeTX/OpenTX based radios, mLRS also translates some common MAVLink messages into telemetry sensors which can be used by the Yaapu Telemetry App as described `here <https://github.com/olliw42/mLRS-docu/blob/master/docs/CRSF.md>`__.

.. image:: ../../../images/mLRS-docu-setup-crsf-telemetry-yaapu-app-02.jpg

The receivers can output RC controls in either SBUS or CRSF(including link quality info) to the autopilot or use embedded MAVLink overrides via only the telemetry connection to the autopilot.

mLRS has been optimized for use with ArduPilot and includes specific support for MAVLink including flow control via RADIO_STATUS messages and optional injection of RC_CHANNELS_OVERRIDE messages allowing a single full duplex serial connection to the flight controller for both RC and MAVLink.

mLRS is rich in features including support for full diversity, 10 model configurations, and OLED display or LUA script for configuration.

You can install the mLRS firmware on one of the supported off the shelf hardware platforms which currently includes the `R9M Tx module <https://www.frsky-rc.com/product/r9m-2019/>`__, `R9MX Rx <https://www.frsky-rc.com/product/r9m-2019/>`__, and R9MM at 868/915 MHz and the `FRM303 Rx/Tx module <https://www.flysky-cn.com/frm303description>`__ at 2.4 GHz.

See the mLRS documentation for the `FrSky R9 <https://github.com/olliw42/mLRS-docu/blob/master/docs/FRSKY_R9.md>`__ or `Flysky FRM303 <https://github.com/olliw42/mLRS-docu/blob/master/docs/FLYSKY_FRM303.md>`__ for instructions on flashing the firmware.

Or, with some soldering skill, you can build one of the DIY board designs which are documented on the  `Github <https://github.com/olliw42/mLRS-hardware>`__ and the `mLRS documentation <https://github.com/olliw42/mLRS-docu/blob/master/README.md>`__

.. image:: ../../../images/Frsky_R9_mLRS.jpg

FrSky R9M Tx module with `M5Stamp pico <https://shop.m5stack.com/products/m5stamp-pico-diy-kit>`__ piggyback `Wireless Bridge <https://github.com/olliw42/mLRS-docu/blob/master/docs/WIRELESS_BRIDGE.md>`__ for Bluetooth or WiFi to GCS pictured with light weight dipole and R9MX receiver
