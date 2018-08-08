.. _debugging-with-gdb:

==================
Debugging with GDB
==================

This page describes how to setup GDB on Linux to debug issues with ArduPilot.

Introduction
============

GDB (the GNU Debugger) "allows you to see what is going on \`inside'
another program while it executes or what another program was doing at
the moment it crashed." which can be useful to debug high level feature or
when investigating very low-level failures with the Pixhawk.

This guide assumes that you have already successfully built the firmware
on your machine following the instructions :ref:`to build the code <building-the-code>`.

In the following page, we will explain how to debug your modification on Linux and Windows with SITL and on Pixhawk directly.

.. toctree::
    :maxdepth: 1

        Debugging with SITL on Linux <debugging-with-gdb-on-linux>
        Debugging with GDB on STM32 <debugging-with-gdb-on-stm32>
