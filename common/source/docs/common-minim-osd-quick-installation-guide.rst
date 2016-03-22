.. _common-minim-osd-quick-installation-guide:

==================================
Minim OSD Quick Installation Guide
==================================

`MinimOSD <https://code.google.com/p/arducam-osd/wiki/minimosd>`__
"On-Screen Display" is a small circuit board that pulls telemetry data
from your APM or Pixhawk flight controller and over-lays it on your
:ref:`First Person View <common-fpv-first-person-view>` monitor.

This article provides brief instructions for how to connect the board.
For more detailed instructions please refer to the `MinimOSD Project wiki <https://code.google.com/p/arducam-osd/wiki/minimosd>`__.

.. note::

   The Minim OSD was designed and programmed by Sandro Benigno and
   Jani Hirvinen. It is `available from jDrones here <http://store.jdrones.com/jD_MiniOSD_V12_p/jdminiosd12.htm>`__.

Overview
========

To connect to Pixhawk, use this `DF13 6-pin cable <http://www.unmannedtechshop.co.uk/df13-6-position-connector-30cm/>`__
to connect to the TELEM2 port. To connect to APM 2.5 and 2.6, use a
5-pin splitter cable that allows the telemetry port to be connected to
both a :ref:`SiK Radio <common-sik-telemetry-radio>` and the MinimOSD.

.. image:: ../../../images/MinimOSD_Pixhawk.jpg
    :target: ../_images/MinimOSD_Pixhawk.jpg

Basic wiring Diagram
====================

The orignal MinimOSD's power setup provides two stages to avoid noises
coming from servos attached to your ArduPilot boards. Those noises could
introduce some glitches on video signal. The independent analog powering
from a dedicated battery will heat the board considerably, but the video
is the most clean as possible from MAX7456.

Maybe you don't need to use the two stages. The way those noises would
impact on the video signal will vary depending on a chain of aspects
like servo's brand, model, cables length, etc. So, try yourself and see
if it's important for your setup.

Here is the basic diagram which uses two stages approach of MinimOSD
board: \ |DiagramaMinimOSD|

Optional setup for critical cooling conditions
==============================================

(Hardware V0.1 and 1.0 only)

The second stage regulator from the MinimOSD boards earlier than V1.1
gets too hot on 12V video setups. If your frame has not a good air flow
for cooling the OSD board you may want to feed the OSD entirely from
APM. Probably it will add some noises from servos, but you'll be more
safe by this way:

.. image:: ../../../images/DiagramaMinimOSD_OP.jpg
    :target: ../_images/DiagramaMinimOSD_OP.jpg

.. |DiagramaMinimOSD| image:: ../../../images/DiagramaMinimOSD.jpg
    :target: ../_images/DiagramaMinimOSD.jpg