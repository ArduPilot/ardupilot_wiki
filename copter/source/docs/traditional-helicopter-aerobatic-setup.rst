.. _traditional-helicopter-aerobatic-setup:

======================================
Traditional Helicopter Aerobatic Setup
======================================

Aerobatic operation is possible in ACRO mode. However, the normal collective pitch setup of -2° to +12° will not allow inverted maneuvers. If desired, the :ref:`H_COL_MAX <H_COL_MAX>` and :ref:`H_COL_MIN <H_COL_MIN>` values cam be setup for -10°or 12° to +10° or +12°. Hover upright will now be between center and high stick and inverted hover between center and low stick.
 
However, if the extremes of -12° to +12° were setup for full aerobatic operation in :ref:`Acro Mode<acro-mode>`, the defaults for ``IM_STB_COL_x`` would yield:

==============      =====
Throttle Stick      Pitch
==============      =====
Low                 -12
Center              +0
High                +12
==============      =====

This will cause extreme frame stress if spooled up at low stick in STABILIZE and will have a jump if hovering in STABILIZE or ACRO and switching to another mode. The STABILIZE curve can be modified using

- :ref:`IM_STB_COL_1<IM_STB_COL_1>` = 42
- :ref:`IM_STB_COL_2<IM_STB_COL_2>` = 65
- :ref:`IM_STB_COL_3<IM_STB_COL_3>` = 76
- :ref:`IM_STB_COL_4<IM_STB_COL_4>` = 100

to provide a similar curve in STABILIZE to that of the narrower(-2° to +12°) collective range and allowing spool up at low stick and no jumps from STABILIZE to other modes. But ACRO will still cause a hover jump if switched to another mode, so be prepared.