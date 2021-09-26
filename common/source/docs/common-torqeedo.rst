.. _common-torqeedo:

========================
Torqeedo Electric Motors
========================

.. image:: ../../../images/torqeedo.jpg

Rover-4.2.0 (and higher) support controlling a single `Torqeedo 1103 electric motor <https://www.torqeedo.com/en/products/outboards/travel/travel-1103-c/M-1151-00.html>`__ (and probably other torqeedo motors) using its custom serial RS485 protocol

.. warning::

   The torqeedo driver is a work-in-progress and the parameters and interface may change.  Currently only one motor can be controlled although this will likely be resolved before the official release.

Some images from this page are courtesy of torqeedo.com

What to Buy
-----------

- Torqeedo motors can be purchased directly from `torqeedo.com <https://www.torqeedo.com/en/products/outboards/travel/travel-1103-c/M-1151-00.html>`__  or one of their `dealers <https://www.torqeedo.com/en/stores>`__
- `Throttle extension cable or motor extension cable <https://www.torqeedo.com/en/search?q=extension%20cable>`__
- Autopilot-to-Torqeedo interface board parts include

    - `RS485 to TTL converter <https://www.amazon.ca/MAX485CSA-Converter-Integrated-Circuits-Products/dp/B06W9H64TN/ref=sr_1_fkmrnull_1?keywords=rs485+to+ttl+lc&qid=1552083892&s=gateway&sr=8-1-fkmrnull>`__
    - `3.3V regulator <https://www.sparkfun.com/products/526>`__
    - `N-Channel MOSFET <https://www.sparkfun.com/products/10213>`__ (only required if using the tiller interface)
    - 3-pin or 4-pin `screw terminal <https://www.sparkfun.com/search/results?term=screw+terminal>`__
    - 6-pin serial cable

AutoPilot connection
--------------------

The autopilot can be connected to either the battery's tiller connector or to the motor connector.  If the torqeedo's built-in battery will be used then the autopilot should be connected to the tiller connector.  If instead a separate battery will be used then the autopilot should be connected to the motor connector (and the built-in battery can be removed)

  .. image:: ../../../images/torqeedo-connector-options.png
      :target: ../_images/torqeedo-connector-options.png
      :width: 400px

An interface board should be assembled using the parts mentioned above and connected as shown below.  The image on the left shows the wiring if the tiller connector is used, the image on the right is for use of the motor connector

  .. image:: ../../../images/torqeedo-autopilot-connection.png
      :target: ../_images/torqeedo-autopilot-connection.png
      :width: 400px

.. warning::

   If the motor connector method is used with non-torqeedo batteries, be sure to use batteries with built-in short circuit protection.  During early development and testing of this interface we managed to burn out a motor which resulted in a short circuit within the motor.  A short circuit especially with large batteries could lead to a fire and serious injury.

Below are pictures of the modified motor extension cable and `Running Electronics <http://www.runele.com/>`__ RS483-to-TTL converter board

  .. image:: ../../../images/torqeedo-motor-cable-small-annotated.jpg
      :target: ../_images/torqeedo-motor-cable-small-annotated.jpg
      :width: 400px

  .. image:: ../../../images/torqeedo-running-electronics-adapter-small.jpg
      :target: ../_images/torqeedo-running-electronics-adapter-small.jpg
      :width: 400px

Configuration
-------------

Please set the following parameters

- :ref:`TRQD_TYPE <TRQD_TYPE>` = 1 (Tiller) if the autopilot will be connected to the tiller connector or 2 (Motor) if connected to the motor connector
- :ref:`SERIAL1_PROTOCOL <SERIAL1_PROTOCOL>` = 39 (Torqeedo) if the Torqeedo is connected to serial port 1.  If another serial port is used please set the appropriate SERIALx_PROTOCOL parameter
- :ref:`SERIAL1_BAUD <SERIAL1_BAUD>` = 19 (19200 bps) if using serial port 1.  If another serial port is used please set the appropriate SERIALx_BAUD parameter instead

If a serial port with flow control pins (clear-to-send and ready-to-send) is used no further parameter settings are required.  If serial port without flow-control is used then please set

- :ref:`TRQD_ONOFF_PIN <TRQD_ONOFF_PIN>` = 54 (AUX5) and connect the autopilot's AUX5 pin to the interface's board's MOSFET
- :ref:`TRQD_DE_PIN <TRQD_DE_PIN>` = 55 (AUX6) and connect the autopilot's AUX6 pin to the RS485<->TTL converter's DE and RE pins

Logging and Reporting
---------------------

By default "TRQD" messages are written to the onboard log every 5 seconds including

- Health : whether the autopilot is successfully controlling the motor
- MotSpeed : the last desired speed sent to the motor as a number from -1000 to +1000
- SuccCnt : the number of messages successfully consumed from the motor (this should rise steadily)
- ErrCnt : the number of bytes or messages that were corrupted or could not be processed (this should be a relatively low number)

This same information can be sent to the ground station by setting :ref:`TRQD_OPTIONS <TRQD_OPTIONS>` = 3

Introduction Video
------------------

.. youtube:: uq1okSejrUE
    :width: 100%

[copywiki destination="rover,dev"]

