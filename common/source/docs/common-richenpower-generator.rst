.. _common-richenpower-generator:

=====================
RichenPower Geneartor
=====================

.. image:: ../../../images/richenpower-generator.png

The RichenPower H2 Hybrid and H2plus Hybrid generators can be used to extend a vehicle's flight time by charging the battery in flight.  Feedback from the generator including RPM, voltage and current allows the operator to monitor the system via the ground stations.

A transmitter's auxiliary switch can be used to control the generator's speed (stop, idle or run).

Where to Buy
------------

The H2 Hybrid and H2plus Hybrid generators can be purchased directly from the `RichenPower webstore <https://www.richenpower.com/shop>`__

Connection and Configuration
----------------------------

The generator's serial output ("#5 wire") should be connected to one of the autopilot serial ports (i.e. Telem2)

One of the autopilot's servo outputs (i.e. "AUX OUT1", aka servo output9) should be connected to the generator's pwm input

Connect to the autopilot with a ground station and set the following parameters:

- :ref:`GEN_TYPE <GEN_TYPE>` = 3 (and reboot the autopilot)
- :ref:`SERIAL2_PROTOCOL <SERIAL2_PROTOCOL>` = 30 (Generator)
- :ref:`SERIAL2_BAUD <SERIAL2_BAUD>` = 9 (9600)
- :ref:`RC9_OPTION <RC9_OPTION>` = 85 (Generator)
- :ref:`SERVO9_FUNCTION <SERVO9_FUNCTION>` = 42 (Generator Control)

Controlling the Generator
-------------------------

Please refer to the generator manual for operating instructions but in short:

- The generator can only be started by pulling the ripcord
- The transmitter's auxiliary switch can change the speed of the generator:

    - Low position stops the generator
    - Middle position for idle
    - High position for run (to charge the battery)

Monitoring from the GCS
-----------------------

Mission Planner has a generator monitor window that appears as soon as GENERATOR MAVLink messages arrive.

.. image:: ../../../images/richenpower-monitor.png
   :width: 350px

Vibration isolation
-------------------

The high vibration from the generator means that :ref:`vibration isolation <common-vibration-damping>` is critical
