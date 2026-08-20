.. _common-firmware-update:

================
Firmware Updates
================

Flying Stable ArduPilot Point Releases
--------------------------------------

The ArduPilot project is very careful about what is merged into our "point releases".

Anything that goes into one has either been evaluated as being strictly limited in what impact it can have on existing systems, or as being an important bugfix.

We encourage users to upgrade to any point release which is released as they sometimes contain very important bugfixes.

Parameter Migration
-------------------

When upgrading you should take a copy of your parameters, and save them with a filename corresponding to the version you are moving from.  If you decide you must return to an earlier firmware these will be invaluable in restoring your vehicle.  However - *do not* ever apply those saved parameter files to newer versions of the firmware!

ArduPilot goes to some effort to make firmware upgrades seamless.  From time to time we need to move parameters around, change their scaling or simply remove them.  We do this automatically on first boot after the firmware is updated.

We can't retain this parameter migration code forever, so there are limits to which versions we guarantee upgrades from and to.  You can still upgrade from older versions to newer versions, but you may find that some functionality changes unexpectedly as you are now using default parameter values rather than your customised values.

If your autopilot falls far behind the modern stable versions you can still get the parameter conversions - you just need to flash intermediate versions - making it a multi-step process.

+--------------+--------------------------+
| New version  | Oldest Migration version |
+==============+==========================+
| 4.8          | 4.2                      |
+--------------+--------------------------+
| 4.7          | 4.2                      |
+--------------+--------------------------+
| 4.6          | 4.1                      |
+--------------+--------------------------+
