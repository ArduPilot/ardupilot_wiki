.. _common-frsky-repurposed:

FrSky Telemetry Using Repurposed Messages
=========================================

For information on how to connect the FrSky equipment together, please go :ref:`here <common-frsky-equipment>`.

For information on how to configure ArduPilot for FrSky telemetry, please go :ref:`here <common-frsky-configMP>`. 

Once your equipment is connected and ArduPilot is configured, follow the instructions below to get standard FrSky telemetry displayed on your RC transmitter.

Protocol information
--------------------

Values that are sent over the FrSky telemetry link by ArduPilot:

+-------------------------------------+---------------------------------------------------------------+
| Taranis telemetry screen identifier |                          Description                          |
+=====================================+===============================================================+
| Fuel                                | Remaining battery capacity %                                  |
+-------------------------------------+---------------------------------------------------------------+
| VFAS                                | Battery voltage                                               |
+-------------------------------------+---------------------------------------------------------------+
| Curr                                | Current consumption                                           |
+-------------------------------------+---------------------------------------------------------------+
| GPS                                 | Latitude/longitude                                            |
+-------------------------------------+---------------------------------------------------------------+
| GSpd                                | GPS groundspeed                                               |
+-------------------------------------+---------------------------------------------------------------+
| GAlt                                | GPS altitude                                                  |
+-------------------------------------+---------------------------------------------------------------+
| Hdg                                 | Yaw angle                                                     |
+-------------------------------------+---------------------------------------------------------------+
| Alt                                 | Navigation altitude (relative to home)                        |
+-------------------------------------+---------------------------------------------------------------+
| Tmp1                                | Control/flight mode                                           |
+-------------------------------------+---------------------------------------------------------------+
| Tmp2                                | GPS status and number of satellites (as num_sats*10 + status) |
+-------------------------------------+---------------------------------------------------------------+

If you installed FrSky sensors on your vehicle, other messages from these may also appear during discovery.

Configuration with OpenTX
=========================

Transmitter set-up
------------------

Please refer to the `OpenTX manual <https://www.gitbook.com/book/opentx/opentx-taranis-manual/details>`__
for how to display values from the FrSky telemetry feed on the RC transmitter's screen.

.. note::

   If upgrading to OpenTX 2.1+ you will need to replace your OpenTX
   2.0 configuration and "discover" your sensors. There are other minor
   "oddities" - for example T1 (flight mode) and Tt2 (number of sats) are
   both called TEMP (switching mode helps you identify which is
   which).

.. image:: ../../../images/Telemetry_FrSky_TXSetup.png
    :target: ../_images/Telemetry_FrSky_TXSetup.png

FrSky telemetry data consists of 16 or 32bit unsigned integers recognized by OpenTX. Standard FrSky telemetry does not include flight controller messages natively. Therefore, less important telemetry data messages (temperature, variometer…) have been repurposed to carry more useful information such as flight mode.

Using telemetry values in OpenTX
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Examples
~~~~~~~~

Variometer
~~~~~~~~~~

In the example below, the variometer function of OpenTX is configured to use the discovered VSpd sensor value. The value is only considered if it is between -10m/s and 10m/s. In the center band -0.5 to 0.5 m/s the variometer will be silent.

.. image:: ../../../images/OpenTX_VarioTelem.png
 :target: ../_images/OpenTX_VarioTelem.png

This example shows how to assign a switch on the Taranis to enable/disable variometer sounds:

.. image:: ../../../images/OpenTX_VarioSwitch.png
 :target: ../_images/OpenTX_VarioSwitch.png

For a more detailed video of how to setup the variometer, you may check out this `video <http://open-txu.org/2-6-02-frsky-variometer-sensor-andrew-newton-02272015-2/>`__
ArduPilot already provides variometer values through the FrSky telemetry link, so the FrSky variometer sensor is not necessary.

GPS
~~~

You can setup the display of the latest transmitted latitude and longitude information; for instance, in case of a crash or fly away, to locate your copter.

.. image:: ../../../images/OpenTX_GPStelem.png
 :target: ../_images/OpenTX_GPStelem.png

When configured to display “GPS,” the custom telemetry screen will show longitude/latitude value pairs as such:

.. image:: ../../../images/OpenTX_GPSdisplay.png
 :target: ../_images/OpenTX_GPSdisplay.png

Configuration with ErSky9x
==========================

For information on how to configure your ErSky9x transmitter (such as a Turnigy 9XR Pro) for FrSky telemetry, please go :ref:`here <common-frsky-telemetry>`.
