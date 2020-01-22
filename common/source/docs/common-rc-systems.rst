.. _common-rc-systems:

================================
Compatible Radio Control Systems
================================

This article provides an overview of the RC Transmitter and Receiver
Systems that can be used with ArduPilot autopilots.

Overview
========

ArduPilot autopilots are compatible with:
    #. PPM-Sum receivers
    #. SBus receivers 
    #. IBUS receivers
    #. Spektrum DSM, DSM2, and DSM-X Satellite receivers
    #. SRXL version 1 and version 2 receivers
    #. Graupner SUM-D

All of the above methods utilize a single serial signal transmission wire for all channels. For traditional single-wire-per-channel (PWM) receivers, a `PPM encoder <http://store.jdrones.com/pixhawk_px4_paparazzi_ppm_encoder_v2_p/eleppmenc20.htm>`__ can be used to convert the receiver outputs to PPM-Sum.

Connecting the Receiver
=======================

ArduPilot auto-detects the protocol of the RC receiver system. For most autopilots there is a pin labeled RCin or SBus input to which the output from the receiver is connected. On many closed source autopilots, other pins are used and are detailed in their board documentation, linked from the :ref:`ArduPilot Hardware<common-autopilots>` page.

In addition, beginning with ArduPilot firmware releases 4.0 and later, any UART RX input will auto-detect the RC receiver, if the serial port protocol to 23 (for example :ref:`SERIAL2_PROTOCOL<SERIAL2_PROTOCOL>` for the TELEM2 UART is used).

To connect a PPM-Sum receiver or an SBus receiver to a Pixhawk, for example, plug the ground (black), power (red) and signal (usually white - orange in the diagram below) wires to the RC pins on the Pixhawk. 

.. image:: ../../../images/RCIN_connection.jpg
    :target: ../_images/RCIN_connection.jpg

.. tip::

   The parameter to enable the SBus output from the PixHawk style autopilots is
   :ref:`BRD_SBUS_OUT<BRD_SBUS_OUT>` . This is only to pass SBus externally to other devices, like servos. Not to connect a receiver to RCin or SBus In.


Radio Control Systems
=====================

Without integrated telemetry:

.. toctree::
    :maxdepth: 1

    FLYSKY <common-flysky-rc>
    Futaba <common-futaba-rc>
    Spektrum <common-spektrum-rc>


With integrated telemetry:

.. toctree::
    :maxdepth: 1

    DragonLink <common-dragonlink-rc>
    FRSky <common-frsky-rc>
    Graupner (HOTT) <common-graupner-rc>
    Multiplex (no support in Ardupilot for M-Link telemetry yet) <common-multiplex-rc>

Multi-Protocol:

.. toctree::
    :maxdepth: 1

    Jumper T16 Pro Transmitter <common-jumperT16pro>



PPM encoders
============

A `PPM Encoder <http://store.jdrones.com/pixhawk_px4_paparazzi_ppm_encoder_v2_p/eleppmenc20.htm>`__ will
allow you to use any conventional RC receiver with only outputs for each channel. Both the new and previous
versions of the *3DR PPM-Sum encoder* (the linked encoder is compatible) are shown
below:

.. figure:: ../../../images/PPM_cables_-_Copy.jpg
   :target: ../_images/PPM_cables_-_Copy.jpg

   Newest 3DR PPM-Sum encoder

.. figure:: ../../../images/PPMEncoderDesc.jpg
   :target: ../_images/PPMEncoderDesc.jpg

   Original 3DR PPM-Sumencoder

There are some downsides of using this encoder:

-  The PPM Encoder does require quite a bit of additional wiring to the receiver.
-  It uses quite a bit of power making it likely you will need to plug
   in your battery while doing radio setup with USB cable in Mission Planner.
-  The encoder also costs as much or more than several of the
   available PPM-Sum receivers including the FrSky Delta 8 below.

There is addition information :ref:`about connecting and configuring the encoder here <common-ppm-encoder>`.

.. note::

   If you are using this PPM Encoder it is important to know that
   when you are calibrating your transmitter you may need
   to hook up your flight battery to the autopilot because the USB port
   alone can't supply enough power.


