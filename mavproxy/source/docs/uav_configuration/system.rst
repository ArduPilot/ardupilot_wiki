===============
System Commands
===============

There are a number of overall system and miscellaneous commands:

reboot
======

Reboots the APM.

setup
=====

Goes into the setup (CLI) mode of the APM.

rc
==

Override a RC (input) channel. This value remains in effect until a
value of 0 is set. Uses the form of ``rc chan value``. Use ``all`` to
set a global value for all RC channels.

.. code:: bash

    rc 1 1000
    rc all 0

time
====

Displays the current time from the autopilot, if supported. The time in
brackets is the ground station time.

script
======

Runs a text file containing MAVProxy commands, much like the :ref:`mavinit file <mavproxy-mavinit>`.

shell
=====

Run a shell command.

status
======

Shows the latest packets received from the autopilot. Useful for reading
the state of the UAV.

exit
====

Exits MAVProxy.

.. note::

    This requires the "requireexit" option to be true in order to work. This can be done via the following command:
    
    .. code:: bash
    
        set requireexit True
    

