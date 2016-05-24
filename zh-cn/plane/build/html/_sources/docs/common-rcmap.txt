.. _common-rcmap:

===========================
RCMAP Input Channel Mapping
===========================

This article shows how to set up a non-standard RC input channel mapping
using the RCMAP feature.

.. note::

   This feature is available in AC3.2.1 and higher (and equivalent
   Plane and Rover versions).

Configuration
=============

By default the RC input channels are:

-  Channel 1: Roll input
-  Channel 2: Pitch input
-  Channel 3: Throttle input
-  Channel 4: Yaw input

These can be changed by setting the ``RCMAP_PITCH``, ``RCMAP_ROLL``,
``RCMAP_THROTTLE`` and ``RCMAP_YAW`` parameters using the *Mission
Planner*'s **Config/Tuning \| Full Parameter Tree** (or Full Parameter
List) as shown below .

.. image:: ../../../images/RCMAP_MPSetup.png
    :target: ../_images/RCMAP_MPSetup.png

After changing any of these parameters the flight controller should be
rebooted.



Additional information about the ``RCMAP`` parameters can be found for the respective platforms in: 
:ref:`Copter Parameters <copter:RCMAP_ROLL>`, :ref:`Plane Parameters <copter:RCMAP_ROLL>`
and :ref:`Rover Parameters <rover:RCMAP_ROLL>`.


