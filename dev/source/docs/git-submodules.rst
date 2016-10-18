.. _git-submodules:

==============
Git Submodules
==============

This page describes how we use `git submodules <https://git-scm.com/book/en/v2/Git-Tools-Submodules>`__ in
the ArduPilot build. Submodules are used to manage external dependencies
in the ArduPilot build, particularly for building for PX4 targets.

Background
==========

*Git submodules* allow us to automatically bring in dependent git trees
in the ArduPilot build. It replaces our old mechanism of having to
separately clone the **ArduPilot/PX4Firmware** and
**ArduPilot/PX4NuttX** tree when developers want to build for PX4
targets.

.. note::

   For now *git submodules* are only used for the PX4 builds. It is
   likely we will start to use submodules for other builds in the future
   (for example, we will probably use them for the mavlink message XML
   files)

See https://git-scm.com/book/en/v2/Git-Tools-Submodules for more
information on *git submodules*.

Submodule approach
==================

The approach we have implemented in ArduPilot is to use a single level
of *git submodules*, with all modules stored in the **modules/**
directory. This approach was chosen as it makes for diagnosis of issues
with submodules somewhat simpler.

This means that the submodules from the upstream PX4Firmware tree are
not used. Instead each required submodule is added as a direct submodule
of the ArduPilot tree.

You may also note that the URLs used for the submodules use the old
``git://`` protocol. This was done to make it less likely we will get
accidental commits on the master repositories while developers are
getting used to *git submodules* (as the ``git://`` protocol is
read-only). Developers with commit access to the submodules should add a
new ardupilot remote with a writeable protocol as needed.

Common errors
=============

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

Updating submodules
-------------------

If you need to manually update submodules you should run the command

::

    git submodule update

from the root of the ardupilot tree. That will check all submodules for
updates in the ardupilot repository and will pull in changes as needed.

Disaster recovery
-----------------

If things have gone very badly wrong with your git tree the simplest
thing to do it to remove the modules/ directory completely. Then do a
new px4 build with

::

    make px4-v2

and the submodules will be automatically reinitialised and updated.
