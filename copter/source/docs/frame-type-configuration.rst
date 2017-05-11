.. _frame-type-configuration:

==================================
Frame Class and Type Configuration
==================================

The :ref:`FRAME_CLASS <copter:FRAME_CLASS>` and :ref:`FRAME_TYPE <copter:FRAME_TYPE>` parameters should be set to match the physical frame being used.

If using the mission planner select Initial Setup, **Mandatory Hardware \| Frame Type**.

.. figure:: ../images/MissionPlanner_Select_Frame-Type.jpg
   :target: ../_images/MissionPlanner_Select_Frame-Type.jpg

If using Copter-3.5 (or higher) firmware a "Frame Class" section should be visible which allows selecting the "Class" of vehicle (Quad, Hexa, Octa, etc).

.. note::

   For Traditional Helicopters, "Heli" should already be selected and it should not be changed.
   For :ref:`Single Copter and Coax Copter <singlecopter-and-coaxcopter>` the :ref:`FRAME_CLASS <copter:FRAME_CLASS>` parameter should be set directly from the Full Parameter List until `this issue <https://github.com/ArduPilot/MissionPlanner/issues/1552>`__ is resolved.

Next select the frame "Type" for your vehicle. The default type is **X**.
Diagrams of most of the available types appear below and on the :ref:`Connect ESC and Motors <connect-escs-and-motors>` page.

For Tricopters, Y6, Traditional Helicopters, SingleCopters and CoaxCopters the frame type is ignored.

If you are using another Ground Station it should be possible to set the :ref:`FRAME_CLASS <copter:FRAME_CLASS>` and :ref:`FRAME_TYPE <copter:FRAME_TYPE>` parameters directly through a parameter update screen.

Motor Order Diagrams
====================

Quad
----

.. image:: ../images/MOTORS_QuadX_QuadPlus.jpg
    :target: ../_images/MOTORS_QuadX_QuadPlus.jpg

.. image:: ../images/MOTORS_Quad_Hb.jpg
    :target: ../_images/MOTORS_Quad_Hb.jpg

.. note::

   A common confusion comes when trying to decide if a particular Quadcopter's Type is "X" or "H".
   The Type depends upon the direction that the motors spin and not on the shape that the arms produce.
   However if the frame physically looks like an "H" and is somewhat flexible (i.e it can be twisted) it's yaw performance may be improved by reversing the motor order and changing the type to "H".
   
Hexa, Octo, Y6
--------------

.. image:: ../images/MOTORS_Hexa-octo-y6.jpg
    :target: ../_images/MOTORS_Hexa-octo-y6.jpg

X8
--

.. image:: ../images/MOTORS_X8.jpg
    :target: ../_images/MOTORS_X8.jpg