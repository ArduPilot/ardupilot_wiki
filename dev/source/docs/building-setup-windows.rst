.. _building-setup-windows:

======================================
Setup the Build Environment on Windows
======================================

There are two options for building on Windows: Windows Subsystem for Linux (WSL) or Cygwin. New developers should use WSL as it offers faster compilation times, greater compatibility, and a larger support base from other developers.


Setup for building with waf using WSL (Windows 10 and 11 only)
--------------------------------------------------------------

#. :ref:`Install WSL as described here <building-setup-windows10>`
#. :ref:`Setup VSCode as described here <editing-the-code-with-vscode>` (optional)


Setup for building with waf using Cygwin
----------------------------------------

.. note::

    Cygwin is not recommended for new developers. However, building using Cygwin is sometimes usefull if you need to compile SITL binaries for Windows. However, you can create Windows SITL executables without using Cygwin, following the instructions given :ref:`here <mp-sitl-custom-code>`.

#. :ref:`Install Cygwin as described here <building-setup-windows-cygwin>`
#. :ref:`Setup Eclipse as described here <building-setup-windows-eclipse>` (optional)
