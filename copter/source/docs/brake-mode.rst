.. _brake-mode:

==========
Brake Mode
==========

This very simple flight mode simply stops the vehicle as soon as
possible using the Loiter controller.  Once invoked, this mode does not
accept any input from the pilot. This mode requires GPS.

Overview
========

When switched on, Brake mode will attempt to stop the vehicle as quickly
as possible.  Good GPS position, :ref:`low magnetic interference on the compass <common-diagnosing-problems-using-logs_compass_interference>`\ and
:ref:`low vibrations <common-diagnosing-problems-using-logs_vibrations>`
are all important in achieving good performance.

If the vehicle is landed in Brake mode it will immediately disarm.

..  youtube:: -Db4u8LJE5w?t=103
    :width: 100%

Controls
========

The pilots controls are ignored in this mode.  The vehicle must be
switched out of this mode before the pilot can re-take control.
