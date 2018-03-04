.. _boat-configuration:

==================
Boat Configuration
==================

.. image:: ../images/boat-icon.png
    :target: ../_images/boat-icon.png

Boats can also be controlled with the standard Rover firmware.  As of firmware version 3.2 the only special behaviour is that in :ref:`Auto <auto-mode>`, :ref:`Guided <guided-mode>`, :ref:`RTL <rtl-mode>` and :ref:`SmartRTL <smartrtl-mode>` modes the vehicle will attempt to maintain its position even after it reaches its destination.

To specify that the vehicle is a boat the :ref:`FRAME_CLASS <FRAME_CLASS>` parameter should be set to 2 (Boat).
