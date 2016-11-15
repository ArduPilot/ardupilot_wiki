.. _common-camera-shutter-triggering-cables-for-purchase:

=================================================
Camera Shutter Trigger Boards/Cables for Purchase
=================================================

This article provides links to cables/boards that can be used for
triggering the camera shutter from ArduPilot.

Overview
========

ArduPilot allows you to :ref:`configure a servo or relay output as the control signal for the camera shutter <common-camera-shutter-with-servo>` so that it can be used in
:ref:`Camera Missions <common-camera-control-and-auto-missions-in-mission-planner>`.
Additional hardware is required to convert the shutter activation signal
to the format expected by the particular camera.

This article contains links to information about camera cables and
boards that can be used for this purpose.

.. tip::

   Please add new boards/cables you discover to this page (or `request the addition <https://github.com/ArduPilot/ardupilot_wiki/issues/new>`__).

Seagull #REC
============

`Seagull #REC <http://www.seagulluav.com/product/seagull-rec/>`__ provides
everything needed to connect a Pixhawk to a supported Sony MultiPort™
equipped camera in order to automate camera shutter triggering. The
current list of compatible cameras :ref:`can be found here <common-camera-shutter-triggering-for-sony-multiport-connectors-using-seagull-map_compatible_cameras>`.

   #REC Camera Board from Seagull

The board supports two trigger modes: AF-T (Autofocus-Trigger, 1 sec
pre-AF then trigger) and IS-T (Instant-Trigger, instant trigger as soon
as camera locks focus). On Sony "Multi" cameras it also supports turning
the camera On/Off.

:ref:`Camera Shutter Triggering for Sony MultiPort Connectors using Seagull #REC <common-camera-shutter-triggering-for-sony-multiport-connectors-using-seagull-map>`
provides a brief overview of how to set up the MAP board. ( the RECs predecessor ) 
More detailed instructions are provided in the `Seagull #REC Manual <http://www.seagulluav.com/manuals/Seagull_REC-Manual.pdf>`__.

TIP:   the #REC superceeds the #MAP and #MAP2 which were it's predecessors.  

SkySight MONO
=============

Farsight's `SkySight MONO <http://skysight.eu/?product=skysight-mono>`__
provides everything needed to connect a Pixhawk to a supported Sony
MultiPort™ equipped camera in order to automate camera shutter
triggering. The current list of compatible cameras is on the `product page <http://skysight.eu/?product=skysight-mono>`__.

.. figure:: ../../../images/SkySightMono.jpg
   :target: ../_images/SkySightMono.jpg

   `SkySightMONO <http://skysight.eu/?product=skysight-mono>`__ and CameraCable

The board supports four camera triggering modes, allowing you to trigger
the shutter in single shot and burst modes, both with and without first
setting the autofocus. The mode used is determined by the servo PWM
output set in the :ref:`Camera Shutter Configuration <common-camera-shutter-with-servo>` in Mission Planner.

:ref:`Camera Shutter Triggering for Sony MultiPort Connectors using SkySight MONO <common-camera-shutter-triggering-for-sony-multiport-connectors-using-skysight-mono>`
provides a brief overview of how to set up the board. More detailed
instructions are provided in the `SkySight MONO User Manual <http://skysight.eu/wp-content/uploads/2015/05/MONO-User-Manual.pdf>`__.

Stratosnapper v2
================

The `Stratosnapper V2 <http://littlesmartthings.com/product/stratosnapper-2/>`__ is a
camera remote-control board which supports a large number of output
types (cables/connectors, infra-red, LANC, etc) and a very broad range
of cameras.

Product information:

-  `Purchase Stratosnapper V2 <http://littlesmartthings.com/product/stratosnapper-2/>`__
-  `Supported cameras <http://littlesmartthings.com/support-documentation/faq/supported-cameras-2/>`__
-  :ref:`Camera Shutter Triggering using Stratosnapper <common-pixhawk-auto-camera-trigger-without-chdk>`
   (wiki guide on how to connect to a Sony NEX5 with an IR trigger.

Detailed information
====================

.. toctree::
    :maxdepth: 1
    
    Seagull #MAP (Sony MultiPort) <common-camera-shutter-triggering-for-sony-multiport-connectors-using-seagull-map>
    SkySight MONO Sony (MultiPort) <common-camera-shutter-triggering-for-sony-multiport-connectors-using-skysight-mono>
    Stratosnapper (IR) <common-pixhawk-auto-camera-trigger-without-chdk>
