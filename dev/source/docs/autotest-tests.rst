.. _autotest-tests:

==============
Autotest Tests
==============

The autotest suite is constantly evolving.  There are a lot of predefined convenience functions available (see `Vehicle tests <https://github.com/ArduPilot/ardupilot/blob/master/Tools/autotest/vehicle_test_suite.py>`__ .  There are also many convenience functions within individual vehicle test suites like the `Plane tests <https://github.com/ArduPilot/ardupilot/blob/master/Tools/autotest/arduplane.py>`__).  These functions can make writing tests fast, and there are a lot of things your test can assume when it starts and stops

This document outlines some of the assumptions you can make and some of the utilities you can take advantage of.

Assumptions Your test Can Make
==============================

Location
--------
For the Copter test suite you can assume the vehicle starts and the same location each time.

Plane starts roughly in the CMAC location.

QuadPlane always starts in the same location in Dalby.

All other tests do not guarantee a starting location or rotation.

Disarmed
--------

You can assume your vehicle starts disarmed (except for AntennaTracker, which is always armed).

RC Inputs
---------

RC inputs are zeroed before your test starts

Contexts
========

Contexts allow test suite state changes to be rolled back conveniently.

Each test gets an implicit context, and you can push any number of contexts onto the stack.  You do not need to wrap your test in a context - you get one for free from the harness.

When you pop a context, various state change are reverted to the way they were before the context was pushed.  Notably:

 - parameters are reverted to their values before the context change
 - message rates set via ``context_set_message_rate_hz`` are reverted
 - filesystem files backed up with ``context_backup_file`` get their original contents back
 - Lua scripts installed with ``install_script_content_context`` are removed

Popping a context may involve rebooting the simulation.  We reboot SITL:
 - if your test reboots the simulation (after resetting parameters to their pre-test values, in case there are reboot-required parameters)
 - if your test set the vehicle home via ``self.set_home(loc)``

Methods
=======

self.reboot_sitl()
------------------

Reboots not just the vehicle but the entire simulation.  Useful for reboot-required parameters.

