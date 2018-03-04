.. _git-submodules:

==============
Git Submodules
==============

   .. image:: ../images/git-submodules.png
       :width: 70%

ArduPilot is dependent upon several external code repositories which are held in **submodules**.  These are forked, cloned and built along with ArduPilot.

- `ChibiOS <https://github.com/ChibiOS>`__
- `UAVCAN <https://github.com/ArduPilot/uavcan>`__
- `PX4Firmware <https://github.com/ArduPilot/PX4Firmware>`__
- `PX4NuttX <https://github.com/ArduPilot/PX4NuttX>`__
- `waf <https://github.com/waf-project/waf>`__

This page describes how we use `git submodules <https://git-scm.com/book/en/v2/Git-Tools-Submodules>`__ in the ArduPilot build.

Submodule approach
------------------

ArduPilot uses a single level of *git submodules*, with all modules stored in the `modules <https://github.com/ArduPilot/ardupilot/tree/master/modules>`__
directory. This approach was chosen as it makes for diagnosis of issues with submodules simpler.  This means that if an external project (i.e. PX4Firmware) has submodules of its own, those submodule appear directly in the `ArduPilot modules directory <https://github.com/ArduPilot/ardupilot/tree/master/modules>`__.

ArduPilot maintains local forks of each external project's repo in order to shielf itself from unexpected changes.

You may also note that the URLs used for the submodules use the old
``git://`` protocol. This was done to make it less likely we will get
accidental commits on the master repositories while developers are
getting used to *git submodules* (as the ``git://`` protocol is
read-only). Developers with commit access to the submodules should add a
new ardupilot remote with a writeable protocol as needed.

Updating your local repo's submodules
-------------------------------------

To manually update submodules use the following command

::

    git submodule update --recursive

Very occasionally a new submodule is added to ArduPilot, after which every developer must run this command:

::

    git submodule init

Common errors
-------------

The following is a list of comment errors and how to deal with them.

Errors with config.mk
---------------------

If you have an existing config.mk you may get an error like this:

::

    ../mk/px4_targets.mk:8: *** NUTTX_SRC found in config.mk - Please see http://dev.ardupilot.com/wiki/git-submodules/. Stop.

That happens because you have previously built with an external
PX4Firmware and PX4NuttX tree, and you need to convert to using
submodules. The simplest way to fix this is to remove your **config.mk**
file as it is no longer needed. If you want to keep the file for some
reason then you can either comment out or removed the lines in
**config.mk** which specify the ``PX4_ROOT``, ``NUTTX_SRC`` and
``UAVCAN_DIR`` variables.

Errors from old PX4Firmware and PX4NuttX trees
----------------------------------------------

You may get warnings like these in your build:

::

    ../mk/px4_targets.mk:23: *** You have an old PX4Firmware tree - see http://dev.ardupilot.com/wiki/git-submodules/
    ../mk/px4_targets.mk:26: *** You have an old PX4NuttX tree - see http://dev.ardupilot.com/wiki/git-submodules/
    ../mk/px4_targets.mk:29: *** You have an old uavcan tree - see http://dev.ardupilot.com/wiki/git-submodules/

This indicates that you have old PX4Firmware or PX4NuttX directories in
../PX4Firmware or ../PX4NuttX. The warning is harmless and won't prevent
you from building. The warning is there so you know that commits and
changes made in those directories won't be used.

Disaster recovery
-----------------

If things have gone very badly wrong with your git tree the simplest
thing to do it to remove the modules/ directory completely from your local repo and run these commands
to reinitialise and update the submodules:

::

    git submodule init
    git submodule update --recursive
