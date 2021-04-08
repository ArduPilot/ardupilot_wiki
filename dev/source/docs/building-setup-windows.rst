.. _building-setup-windows:

======================================
Setup the Build Environment on Windows
======================================

There are two options for building on Windows: Windows Subsystem for Linux (WSL) or Cygwin. New developers should use WSL as it offers faster compilation times, greater compatibility, and a larger support base from other developers.

Setup for building with waf using WSL (Windows10 only)
------------------------------------------------------

#. :ref:`Install WSL as described here <building-setup-windows10>`

.. note::

    Cygwin is not reccomended for new developers. However, building using Cygwin is necessary if you need to compile SITL for direct use inside Mission Planner.

Setup for building with waf using Cygwin
----------------------------------------

#. :ref:`Install cygwin as described here <building-setup-windows-cygwin>`

#. :ref:`Setup Eclipse as described here <building-setup-windows-eclipse>` (optional)
