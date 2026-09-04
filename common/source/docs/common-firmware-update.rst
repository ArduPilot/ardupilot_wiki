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
| 4.8(latest)  | 4.2                      |
+--------------+--------------------------+
| 4.7          | 4.2                      |
+--------------+--------------------------+
| 4.6          | 4.1                      |
+--------------+--------------------------+
| 4.5          | 4.1                      |
+--------------+--------------------------+
| 4.4          | 4.1                      |
+--------------+--------------------------+

Note that not every vehicle type has a release for every version number - Sub, for example, has no 4.2, 4.3, 4.4 or 4.6 release.  Use the oldest release your vehicle type has which is no older than the version in the table above; a Sub user on 4.1 would move to 4.5, then to 4.7.
