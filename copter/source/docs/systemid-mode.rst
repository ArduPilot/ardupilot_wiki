.. _systemid-mode:

==========================
System Identification Mode
==========================

This mode is for advanced users and provides a means to generate mathematical models of the vehicles flight behavior for model generation. It places the vehicle into a STABILIZE like mode, and generates bursts of signals ("chirps") injected into the control loops at various points and logs the results for math analysis and model generation later.


Enabling the Mode
=================

If the :ref:`SID_AXIS<SID_AXIS>` is non-zero, then this modes associated parameters will become visible on the next parameter refresh and entry into the mode will be allowed. Otherwise, an error message will be logged and sent to the ground control station indicating that mode entry has been prohibited.

Further Information
===================

Information on this modes parameters and operation are given in the links below:

.. toctree::
    :maxdepth: 1

    System ID Mode Operation <systemid-mode-operation>