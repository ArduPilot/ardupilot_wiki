.. _common-simplebgc-gimbal:

=====================================
SimpleBGC Gimbal (aka AlexMos gimbal)
=====================================

The SimpleBGC is a popular brushless gimbal controller that can communicate with ArduPilot (Copter, Plane and Rover) using a custom serial protocol.
More details on the capabilities of this gimbal can be found at `basecamelectronics.com <https://www.basecamelectronics.com/>`__

.. note::

   Support for this gimbal is included in Copter 3.4 (and higher). 

Where to Buy
============

The SimpleBGC controller and accompanying 2-axis and 3-axis gimbals can be purchased from `basecamelectronics.com <https://www.basecamelectronics.com/>`__ and many other retailers.

Connecting the gimbal to the Flight Controller
==============================================

.. image:: ../../../images/simplebgc-gimbal-pixhawk.png
    :target: ../_images/simplebgc-gimbal-pixhawk.png

Although the SimpleBGC can be connected using PWM (similar to the Tarot gimbal) we recommend using the serial interface connected to one of the flight controller's Serial/Telemetry ports like Telem2 as shown above.

Setup through the Ground Station
================================

Set the following parameters through your ground station and then reboot the flight controller:

- :ref:`MNT_TYPE <MNT_TYPE>` to 3 / "AlexMos-Serial"
- :ref:`SERIAL2_PROTOCOL <SERIAL2_PROTOCOL>` to 7 / "AlexMos Gimbal Serial"  (Notee "SERIAL2" should be "SERIAL1" if using Telem1 port, SERIAL4 if using Serial4/5, etc)

If you are unable to connect you may wish to set the following parameters although normally this should not be required:

- :ref:`SERIAL2_BAUD <SERIAL2_BAUD>` to 115 (means use serial baudrate of 115200)
- :ref:`BRD_SER2_RTSCTS <BRD_SER2_RTSCTS>` to 0 to disable flow control on Telem2 (use BRD_SER1_RSCTS if connecting to Serial1, Serial4/5 never uses flow control)

The gimbal's maximum lean angles can be set using these parameters:

- :ref:`MNT_ANGMIN_ROL <MNT_ANGMIN_ROL>`, :ref:`MNT_ANGMAX_ROL <MNT_ANGMAX_ROL>` to -3000 and 3000 to limit the roll angle to 30 degrees in each direction
- :ref:`MNT_ANGMIN_TIL <MNT_ANGMI_TIL>`, :ref:`MNT_ANGMAX_TIL <MNT_ANGMAX_TIL>` to -9000 and 0 to limit the gimbal to point between straight down (-90 degrees) and straight forward (0 degrees)

To control the gimbal's lean angles from a transmitter set:

- :ref:`MNT_RC_IN_TILT <MNT_RC_IN_TILT>` to 6 to control the gimbal's tilt (aka pitch angle) with the transmitter's Ch6 tuning knob

For a 3-axis gimbal with 360 degrees of yaw set:

- :ref:`MNT_ANGMIN_PAN <MNT_ANGMIN_PAN>`, :ref:`MNT_ANGMAX_PAN <MNT_ANGMAX_PAN>` to -18000 and 18000 to get a full 360 degrees of yaw range

Testing the gimbal
==================

For instructions for testing the gimbal moves correctly please check the :ref:`similar section for the Tarot gimbal <common-tarot-gimbal_testing_the_gimbal_moves_correctly>`.
