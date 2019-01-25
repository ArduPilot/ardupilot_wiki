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

Seagull #IR
============

`Seagull #IR <https://www.seagulluav.com/product/seagull-ir/>`__ is an infrared camera trigger device. It supports a wide range of camera brands and models. Check the full compatibility list at `Seagull #IR page <https://www.seagulluav.com/product/seagull-ir/>`__.

.. figure:: https://www.seagulluav.com/wp-content/uploads/2016/03/SIRU-1000-01-570x570.png
   :target: https://www.seagulluav.com/wp-content/uploads/2016/03/SIRU-1000-01-570x570.png
   
   #IR Infrared Camera Shutter Controller from Seagull UAV

The board supports the following two modes: 

-  **Picture** (Shutter trigger) 
-  **Video Record** (only for **Sony** cameras)

More detailed instructions on how to setup #IR are provided in the `Seagull #IR Manual <https://www.seagulluav.com/manuals/Seagull_IR-Manual.pdf>`__ as well as `#IR Support <https://www.seagulluav.com/seagull-ir-support/>`__ page.

Seagull #MAP2
============

`Seagull #MAP2 <https://www.seagulluav.com/product/seagull-map2/>`__ provides
everything needed to connect a Pixhawk to a camera in order to automate camera shutter triggering. The
current list of compatible cameras can be found on `Seagull MAP Cable Finder <https://www.seagulluav.com/map-cable-finder/>`__.

.. figure:: https://www.seagulluav.com/wp-content/uploads/2016/03/SMAP-1100_01.png
   :target: https://www.seagulluav.com/wp-content/uploads/2016/03/SMAP-1100_01.png
   
   #MAP2 Camera Shutter Controller from Seagull UAV

The board supports two shutter trigger modes: 

-  **AF-T** ("AutoFocus-Trigger", with 1 second pre-focus followed by trigger) 
-  **IS-T** ("Instant-Trigger", instant trigger as soon as camera locks focus)

Seagull #MAP2 also supports **"Camera ON/OFF"** - which is exclusive to **Sony MULTI** cameras.

More detailed instructions on how to setup #MAP2 are provided in the `Seagull #MAP2 Manual <https://www.seagulluav.com/manuals/Seagull_MAP2-Manual.pdf>`__ as well as `#MAP2 Support <https://www.seagulluav.com/seagull-map2-support/>`__ page.

Link to Ardupilot page for `Seagull #MAP2 <http://ardupilot.org/copter/docs/common-camera-shutter-triggering-using-seagull-map2.html?highlight=map2/>`__ .


Seagull #MAP-X2
============

`Seagull #MAP-X2 <https://www.seagulluav.com/product/seagull-map-x2/>`__ is a precision camera shutter trigger and logger. It is very easy to integrate into a Pixhawk in order to automate camera shutter triggering and log the precise co-ordinates of each photo for geotagging purposes. 

Seagull #MAP-X2 utilizes event based logging for extra precision by using `#SYNC2 <https://www.seagulluav.com/product/seagull-sync2/>`__

The current list of compatible cameras can be found on `Seagull MAP Cable Finder <https://www.seagulluav.com/map-cable-finder/>`__.

.. figure:: https://www.seagulluav.com/wp-content/uploads/2018/09/SMX2-1001_01-570x570.png
   :target: https://www.seagulluav.com/wp-content/uploads/2018/09/SMX2-1001_01-570x570.png
   
   #MAP-X2 Camera Shutter Controller / Logger from Seagull UAV

The board supports 4 shutter trigger modes: 

-  **ACT** ("AutoCustomTrigger", user is able to define required focus time and then trigger) 
-  **PWM** ("PWM", fully customisable PWM trigger for cameras that utilze PWM triggering)
-  **SST** (Read event signal then trigger)
-  **Timelapse** (User defined interval triggering)

Seagull #MAP-X2 also supports **"Camera ON/OFF"** - which is exclusive to **Sony MULTI** cameras.

More detailed instructions on how to setup #MAP-X2 are provided in the `Seagull #MAP-X2 Manual <https://www.seagulluav.com/manuals/Seagull_MAP-X2-Manual.pdf>`__ as well as `#MAP-X2 Support <https://www.seagulluav.com/seagull-map-x2-support/>`__ page.

Seagull #REC
============

`Seagull #REC <http://www.seagulluav.com/product/seagull-rec/>`__ provides
everything needed to connect a Pixhawk to a supported Sony MultiPort™
equipped camera in order to automate camera shutter triggering. The
current list of compatible cameras `can be found here <https://www.seagulluav.com/product/seagull-rec/>`__.

.. figure:: https://www.seagulluav.com/wp-content/uploads/2015/11/SREC-1000_1_no_text.png
   :target: https://www.seagulluav.com/wp-content/uploads/2015/11/SREC-1000_1_no_text.png
   
   #REC Camera Controller from Seagull UAV

The board supports three shutter trigger modes: 

-  **AF-T** ("AutoFocus-Trigger", with 1 second pre-focus followed by trigger) 
-  **IS-T** ("Instant-Trigger", instant trigger as soon as camera locks focus)
-  **Manual** ("Three stage trigger control", neutral-focus-trigger)

Seagull #REC also supports **"Video RECORD", "ZOOM", "Timelapse Triggering" and "Camera ON/OFF"**.

More detailed instructions on how to setup #REC are provided in the `Seagull #REC Manual <https://www.seagulluav.com/manuals/Seagull_REC-Manual.pdf>`__ as well as `#REC Support <https://www.seagulluav.com/seagull-rec-support/>`__ page.

TIP:   The #REC is only compatible with "**Sony MULTI**" cameras.  

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
