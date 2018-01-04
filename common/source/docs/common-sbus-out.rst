.. _common-sbus-output:

=============================
SBus Servo Out on Serial Port
=============================

This article explains how to configure a serial port to control `S.BUS <http://www.futabarc.com/sbus/>`_ servos. The SBus protocol supports PWM values for 16 channels, and these are mapped to ArduPilot's servo channels 1-16. The transmission rate is 100K baud with inverted logic levels (unidirectional: output only), and an inverting cable will be required to use a standard serial port. A simple NPN transistor inverter will suffice:

.. image:: ../../../images/sbus/sbus_inverter.png

Set the protocol for the desired serial port to SBUS1. For example, to use port Telem2 (serial2) set parameter :ref:`SERIAL2_PROTOCOL` to 15. When SBus protocol is selected, the port's baud rate parameter (in this case :ref:`SERIAL2_BAUD`) will automatically be set to 100,000. Parameter :ref:`SERVO_SBUS_RATE` defaults to 50 Hz, but may be set to any value in the range of [25,250] Hz. Beware that some SBus to PWM adapters e.g. `FrSky <http://alofthobbies.com/frsky-sbus-cppm-decoder-with-pins.html>`__ generate a fixed PWM output rate (~170 Hz) that may damage analog servos. Digital servos and those designed with SBus inputs should handle high frame rates with no problems.

