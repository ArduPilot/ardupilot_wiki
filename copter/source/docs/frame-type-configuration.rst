.. _frame-type-configuration:

==================================
Frame Class and Type Configuration
==================================

The :ref:`FRAME_CLASS <copter:FRAME_CLASS>` and :ref:`FRAME_TYPE <copter:FRAME_TYPE>` parameters should be set to match the physical frame being used.  See the :ref:`Connect ESCs and Motors <connect-escs-and-motors>` page for the list of supported multicopter frames

If using the mission planner select Initial Setup, **Mandatory Hardware \| Frame Type**.  If you are using another Ground Station it should be possible to set the :ref:`FRAME_CLASS <copter:FRAME_CLASS>` and :ref:`FRAME_TYPE <copter:FRAME_TYPE>` parameters directly through a parameter update screen.

.. figure:: ../images/MissionPlanner_Select_Frame-Type.jpg
   :target: ../_images/MissionPlanner_Select_Frame-Type.jpg

If using Copter-3.5 (or higher) firmware a "Frame Class" section should be visible which allows selecting the "Class" of vehicle (Quad, Hexa, Octa, etc).

.. note::

   For Traditional Helicopters, "Heli" should already be selected and it should not be changed.
   For :ref:`Single Copter and Coax Copter <singlecopter-and-coaxcopter>` the :ref:`FRAME_CLASS <copter:FRAME_CLASS>` parameter should be set directly from the Full Parameter List until `this issue <https://github.com/ArduPilot/MissionPlanner/issues/1552>`__ is resolved.

Next select the frame "Type" for your vehicle. The default type is **X**.

For Tricopters, Y6, Traditional Helicopters, Bicopters, SingleCopters and CoaxCopters the frame type is ignored.

Motor Order Diagrams
====================

See the :ref:`Connect ESCs and Motors <connect-escs-and-motors>` page for the list of supported multicopter frames
