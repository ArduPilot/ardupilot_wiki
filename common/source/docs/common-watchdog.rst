.. _common-watchdog:


Independent Watchdog
====================

In 4.0 and later firmware revisions, the hardware independent cpu watchdog can be enabled by setting :ref:`BRD_OPTIONS<BRD_OPTIONS>` = 1. The cpu will be reset if a peripheral or code "hangs" the cpu, and will restart the cpu. This may or may not save the vehicle if in flight or motion.


.. youtube:: ZGuTIPLI_e0