.. _mavproxy-mavinit:

===============
Startup Scripts
===============

MAVProxy is capable of executing a script (of MAVProxy commands) on
startup. This can save the effort of (re)setting up the MAVProxy
environment for each flight.

A script (:file:`mavinit.scr`) should be placed in an aircraft directory (ie.
:file:`./Aircraftname/mavinit.scr`) containing the commands. Any normal
MAVProxy commands can be placed in here. Note that the --aircraft option
must be used, in order for MAVProxy to find the script in the
appropriate aircraft directory.

Alternatively, a :file:`.mavinit.scr` can be placed in the user's home directory
(ie. :file:`/home/username` in Linux or
:file:`C:\\Users\\username\\AppData\\Local\\MAVProxy` in Windows) and will be
loaded with MAVProxy regardless of the --aircraft option.

An example script can be found `here <https://github.com/ArduPilot/MAVProxy/blob/master/windows/mavinit.scr>`_.

In this particular script, aliases are used as shortcuts to common
commands.

For example the following alias definitions in :file:`mavinit.scr`
will allow the user to graph the current battery level by typing
``gbatt`` instead of ``graph SYS_STATUS.current_battery``.

.. code:: bash

    @alias add g graph
    @alias add gbatt g SYS_STATUS.current_battery`` 

