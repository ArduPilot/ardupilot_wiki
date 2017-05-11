.. _common-choosing-a-flight-controller:

============================
Choosing a Flight Controller
============================

ArduPilot runs on many different flight controller boards, the most
important of which are linked from the topic :ref:`AutoPilot Hardware Options <common-autopilots>`.

Selecting the right board depends on the physical restraints of the
vehicle and the applications that you want to run. Broadly speaking:

-  :ref:`Pixhawk <common-pixhawk-overview>` is highly recommended for
   general use.
-  :ref:`Pixracer <common-pixracer-overview>` is recommended for small
   frames that require no more than 6 PWM outputs.
-  :ref:`NAVIO+ <common-navio-overview>` or
   :ref:`Erle-Brain <common-erle-brain-linux-autopilot>` Linux Autopilots
   should be considered for UAV Vision applications.

.. tip::

   There are also numerous clones and minor variants of the boards
   linked above. Many of these may be perfectly capable replacements.

[site wiki="copter, plane" heading="off"]

.. note::

   The APM2.6 board is no longer supported for Copter or Plane. The
   last firmware builds that can be installed are AC v3.2.1 and Plane
   3.3.0.

[/site]

.. image:: ../../../images/ChooseAFlightController_TitleImage4.jpg
    :target: ../_images/ChooseAFlightController_TitleImage4.jpg
