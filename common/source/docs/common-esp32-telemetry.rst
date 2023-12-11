.. _common-esp32-telemetry:
[copywiki destination="plane,copter,rover,blimp"]
======================
ESP32 WiFi telemetry
======================

The ESP32 are readily available Wi-Fi modules with full TCP/IP stack and
microcontroller capability. They offer dedicated UART, SPI and I2C
interfaces. They can be used with any ArduPilot autopilot controller.

DroneBridge for ESP32
---------------------

| **DroneBridge for ESP32 offers a transparent and bi-directional serial
  to WiFi bridge.**
| Using WiFi protocol does not offer the same range as the other
  DroneBridge implementations. Typical WiFi range is ~50m-200m depending
  on the antennas. High gain directional antennas would offer even more
  range.

.. image:: https://raw.githubusercontent.com/DroneBridge/ESP32/master/wiki/db_ESP32_setup.png
   :alt: DroneBridge for ESP32 connection concept

Recommended Hardware
--------------------

Almost every ESP32 development board is capable to run DroneBridge for
ESP32. Boards and modules with an external antenna connector are
recommended, since those will offer more range.

.. warning:: Most modules support 3.3V input (only), while some autopilots serial ports provide only 5V . You will need to check compatibility and step down the voltage if needed. It is not generally recommended to use the autopilot's 3.3V supply unless you are certain it can provide enough current for the ESP32 board you are using.

Some examples for modules and DevKits that accept 3.3V supply:

-  AZDelivery DevKit C
-  `TinyPICO - ESP32 Development Board - V2 <https://www.adafruit.com/product/4335>`_
-  `Adafruit HUZZAH32 – ESP32 Feather Board <https://www.adafruit.com/product/3405>`_
-  `Adafruit AirLift – ESP32 WiFi Co-Processor Breakout Board <https://www.adafruit.com/product/4201>`_ (requires FTDI adapter for flashing firmware)
-  `Adafruit HUZZAH32 <https://www.adafruit.com/product/4172>`_ (requires FTDI adapter for flashing firmware)
-  ESP32-WROOM-32UE (module only - requires custom PCB)
-  ESP32-WROOM-32E  (module only - requires custom PCB)

.. note::
  NodeMCU style DevKit Boards with an IPEX port for an external antenna
  often also offer an onboard antenna that is activated by default. You
  may need to re-solder a resistor to activate the external antenna port.

Downloading and Flashing the Firmware
-------------------------------------

`Download the firmware from the GitHub repository`_ and `follow the
flashing instructions there`_. They are always up to date.

.. note::

  `Follow the flashing instructions inside the GitHub Repository.`_ The
  exact parameters may differ from release to release of DroneBridge for
  ESP32.

For convenience reasons some short instructions are given here:

-  `Download the pre-compiled firmware binaries`_
-  Connect your DEVKit to your computer via USB/Serial bridge (most
   DevKits already offer a USB port for flashing and debugging)
-  Erase the flash and flash the DroneBridge for ESP32 firmware onto
   your ESP32

   -  Using `Espressif Flash Download Tool`_ (Windows only)
   -  Using esp-idf/esptool (all platforms)

-  Power Cycle the ESP32
-  Connect to the "DroneBridge for ESP32" WiFi network and configure
   the firmware for your application

Configuring DroneBridge for ESP32
---------------------------------

You can change the default configuration via the Webinterface.
Connect to the ESP32 via WiFi and enter ``dronebridge.local``, ``http://dronebridge.local`` or ``192.168.2.1`` in the address
bar of your browser.

Default Configuration
~~~~~~~~~~~~~~~~~~~~~

-  SSID: ``DroneBridge for ESP32``
-  Password: ``dronebridge``
-  Transparent/MAVLink
-  UART baud rate ``115200``
-  UART TX pin ``17``
-  UART RX pin ``16``
-  Gateway IP: ``192.168.2.1``

Custom Settings & Webinterface
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| You can change the default configuration via the Webinterface.
| Connect to the ESP32 via WiFi and enter ``dronebridge.local``,
  ``http://dronebridge.local`` or ``192.168.2.1`` in the address bar of
  your browser.

.. image:: https://raw.githubusercontent.com/DroneBridge/ESP32/master/wiki/dbesp32_webinterface.png
   :alt: DroneBridge for ESP32 Webinterface

.. note::

  Some settings require you to reboot the ESP32 to take effect.

Wiring
======

Wiring is very simple and mostly the same for all devices connected to
any serial port (eg TELEM1 or TELEM2) of the autopilot. This guide does not go into
detail here, but provides an outline for wiring below.

-  Connect UART of ESP32 to a UART of your autopilot (e.g. TELEM
   1 or TELEM 2 port). Make sure the voltage levels match! Most ESP32
   DevKits can only take 3.3V!

   -  TX to RX
   -  RX to TX
   -  GND to GND
   -  Stable 3.3V or 5V power supply to the ESP32 (depending on the
      available inputs of your DevKit and capabilities of the autopilot)

-  Set the autopilot port to MAVLINK 1 or 2 protocol.

Some manufacturers of ESP32 DevKits have wrong labels for the
pins on their products. Make sure that the PINs on your board are
labeled correctly if you encounter issues.

Make sure to always follow the instructions of the ESP32 board manufacturer when it comes to wiring. Especially the power supply.

.. image:: https://raw.githubusercontent.com/DroneBridge/ESP32/master/wiki/Pixhawk_wiring.png
   :alt: Example wiring of autopilot to ESP32


ArduPilot configuration
=======================

Configure the UART of the autopilot that is wired to the ESP32. The default configuration of DroneBridge is:

-  Protocol: MAVLink (v1 or v2)
-  Baudrate: 115200 baud

If connected to Serial2 these parameters should be set on the autopilot (if using another serial port, replace the "2" in the parameter name with the serial port's number):

- :ref:`SERIAL2_PROTOCOL <SERIAL2_PROTOCOL>` = 2 (MAVLink2) or 1 (MAVLink1)
- :ref:`SERIAL2_BAUD <SERIAL2_BAUD>` = 115 (115200 baud)

If you have problems connecting, it may help to set :ref:`BRD_SER2_RTSCTS <BRD_SER2_RTSCTS>` = 0 to disable flow control although this is not normally necessary


Connecting to the GCS
=====================

The following connection options are available:

-  UDP unicast on port ``14550`` to all connected devices.
-  TCP on port ``5760``

DroneBridge for ESP32 will automatically forward all data to all
connected WiFi devices via UDP to port 14550. QGroundControl or Mission Planner should
auto-detect the connection and no further actions should be necessary.


Toubleshooting
==============

-  Always erase the flash of the ESP32 before flashing a new
   release/firmware
-  Check if the pins on your ESP board are labeled correctly.
-  Enter the IP address in your browsers address bar
   ``http://192.168.2.1``. No https supported! You may need to
   disconnect from the cellular network when using a phone to be able to
   access the webinterface.
-  If your network is operating in the same IP range as DB for ESP32 you
   need to change the Gateway IP address in the Webinterface to
   something like ``192.168.5.1``.

API
===

DroneBridge for ESP32 offers a REST:API that allows you to read and
write configuration options. You are not limited to the options
presented by the Webinterface (e.g. baud rates). You can use the API to
set custom baud rates or to integrate the system into your own setup.

**To request the settings**

::

   http://dronebridge.local/api/settings/request

**To request stats**

::

   http://dronebridge.local/api/system/stats

**Trigger a reboot**

::

   http://dronebridge.local/api/system/reboot

**Trigger a settings change:** Send a valid JSON

.. code:: json

   {
     "wifi_ssid": "DroneBridge ESP32",
     "wifi_pass": "dronebridge",
     "ap_channel": 6,
     "tx_pin": 17,
     "rx_pin": 16,
     "telem_proto": 4,
     "baud": 115200,
     "msp_ltm_port": 0,
     "ltm_pp": 2,
     "trans_pack_size": 64,
     "ap_ip": "192.168.2.1"
   }

to

::

   http://dronebridge.local/api/settings/change


.. _Download the firmware from the GitHub repository: https://github.com/DroneBridge/ESP32/releases
.. _follow the flashing instructions there: https://github.com/DroneBridge/ESP32#installationflashing-using-precompiled-binaries
.. _Follow the flashing instructions inside the GitHub Repository.: https://github.com/DroneBridge/ESP32#installationflashing-using-precompiled-binaries
.. _Download the pre-compiled firmware binaries: https://github.com/DroneBridge/ESP32/releases
.. _Espressif Flash Download Tool: https://www.espressif.com/en/support/download/other-tools
