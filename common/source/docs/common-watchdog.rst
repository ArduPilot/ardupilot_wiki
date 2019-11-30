.. _common-watchdog:

====================
Independent Watchdog
====================

In ArduPilot 4.0 and later firmware revisions, the autopilot cpu's internal independent watchdog has been enabled. It can be disabled by setting :ref:`BRD_OPTIONS<BRD_OPTIONS>` = 0. The cpu will be reset if a peripheral or code "hangs" the cpu, and will restart the cpu. This may or may not save the vehicle if in flight or motion.


.. youtube:: ZGuTIPLI_e0

Determining that a Watchdog Reset Occurred
==========================================

One way is by looking at the dataflash logs. If the log is filtered to show only the "MSG" messages it can be seen that some include the word, "watchdog". This is a clear indication that the previous log or flight ended with a watchdog reset.


.. image:: ../../../images/watchdog.png
     :target: ../_images/watchdog.png

