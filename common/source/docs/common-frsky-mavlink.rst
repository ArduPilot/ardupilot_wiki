.. _common-frsky-mavlink:

MavLink to FrSky Converters
===========================

This section contains hardware MAVLink to FrSky converter solutions.

Please feel free to add your own solutions (`or ask us to <https://github.com/ArduPilot/ardupilot/issues/new>`__).

`DIY solution for the APM2.x here <http://diydrones.com/forum/topics/amp-to-frsky-x8r-sport-converter>`__

.. _common-frsky-telemetry_apm_mavlink_to_frsky_smartport_converter_airborne_projects:

APM MavLink to FrSky SmartPort Converter (Airborne Projects)
------------------------------------------------------------

Airborne Project's `APM MavLink to FrSky SmartPort Converter <https://www.airborneprojects.com/product/apm-mavlink-to-frsky-smartport-converter/>`__
converts MAVLink messages to FrSkySmartPort format. It can directly be
connected to the Taranis Radio. You only have to load the Taranis
telemetry modules and configure in Mission Planner. No soldering
required!

.. figure:: https://www.airborneprojects.com/wp-content/uploads/2015/08/converter_1-500x500.jpg
   :target:  https://www.airborneprojects.com/product/apm-mavlink-to-frsky-smartport-converter/

   AirborneProjects: APM MavLink to FrSky SmartPort Converter

The converter includes all needed cables. It features a hardware
modified version of the Arduino Nano and be powered directly from the
Taranis receiver.

For more information see the `QuickStart Guide <https://www.airborneprojects.com/wp-content/uploads/2016/02/Quick-Start-Guide.pdf>`__
(www.airborneprojects.com).

APM MavLink to FrSky SmartPort Converter (MavLink_FrSkySPort)
-------------------------------------------------------------

There are several open source projects such as 
`MavLink_FrSkySPort <https://github.com/Clooney82/MavLink_FrSkySPort/wiki>`__
project, the `Scottflys <http://openbrainiacs.com/tiki-index.php?page=Teensy+Telemetry+Project>`__ or the `Athertop <https://github.com/athertop/MavLink_FrSkySPort>`__ project, that use the Teensy USB Development board to convert MAVLink
messages to FrSkySmartPort format so that ArduPilot telemetry can be
displayed on an FrSky transmitter.

.. image:: https://raw.githubusercontent.com/Clooney82/MavLink_FrSkySPort/s-c-l-v-rc-opentx2.1/images/Basic%20Wiring%20-%20Teensy3.jpg
    :target:  https://raw.githubusercontent.com/Clooney82/MavLink_FrSkySPort/s-c-l-v-rc-opentx2.1/images/Basic%20Wiring%20-%20Teensy3.jpg

.. note::

   This solution is one of the most versatile solutions available,
   and is the inspiration of many similar solutions 
   (including :ref:`Airborne's above <common-frsky-telemetry_apm_mavlink_to_frsky_smartport_converter_airborne_projects>`).
   It is however not a "commercial" solution and does require some soldering, but can be extended even to connect RGB Led strips like `Scottflys LED Extension <http://openbrainiacs.com/tiki-index.php?page=Teensy%20Telemetry%20LED%20Extension>`__.

Information about the circuits and software can be found on the each project link above.
