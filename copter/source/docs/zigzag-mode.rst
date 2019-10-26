.. _zigzag-mode:

===========
ZigZag Mode
===========

ZigZag mode is a semi-autonomous mode designed to make it easier for a pilot to fly a vehicle back and forth across a field which can be useful for crop spraying

..  youtube:: _qNmLX4QmPU
    :width: 100%

.. note::

   ZigZag is available in Copter-4.0 (and higher)

The way it works is:

- A two or preferably three position :ref:`auxiliary switch <channel-7-and-8-options>` is set to "ZigZag SaveWP" (i.e. :ref:`RC7_OPTION <RC7_OPTION>` = 61)
- The pilot arms the vehicle in :ref:`Loiter <loiter-mode>` mode, takes off and then changes to ZigZag mode (in the future we may make it possible to arm and take-off in ZigZag mode)
- The vehicle is flown manually (it flies like :ref:`Loiter <loiter-mode>`) to one side of the field and then the auxiliary switch is moved to the highest or lowest position (it doesn't matter which) to record that side
- The vehicle is flown to the other side of the field and the switch is moved to the opposite position
- The switch can now be used to start the vehicle flying autonomously (at the current height) to either side of the field.  Once the vehicle reaches the other side it will revert to manual control.  The pilot can also regain manual control by moving the auxiliary switch to the middle position or by changing the flight mode.
- If a downward facing :ref:`range finder <common-rangefinder-landingpage>` is used, the vehicle will follow the terrain when flying.  If the range finder becomes unhealthy while traversing from one side to the other, the vehicle will revert to manual control and come to a stop.

.. image:: ../images/zigzag-mode.png
   :target: ../_images/zigzag-mode.png
   :width: 450px

When flown manually ZigZag uses :ref:`Loiter mode's <loiter-mode>`  parameters.  For example the top speed can be configured with :ref:`LOIT_SPEED <LOIT_SPEED>`.

When autonomously flying from one side of the field to the other, the :ref:`Auto mode <auto-mode>` parameters are used.  For example the top speed can be configured with :ref:`WPNAV_SPEED <WPNAV_SPEED>`..

