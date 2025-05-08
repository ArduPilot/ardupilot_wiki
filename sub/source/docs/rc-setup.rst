.. _rc-setup:

=============
Radio Control
=============

Due to the RF attenuation of even a short distance of water above a submersible, radio control must be accomplished by floating tethered bouy to the vehicle which contains the RF components and antenna. This applies also to RF based telemetry, whether independent of, or integrated into, the radio control system.

To enable radio control, the :ref:`RC_PROTOCOLS<RC_PROTOCOLS>` parameter must be set to a non-zero value, "1" is usually used. If rc control of the flight mode is desired, in addition to axis/directional control, the :ref:`FLTMODE_CH<FLTMODE_CH>` should be set to the RC channel that will control flight modes, if this is desired.

Any ArduPilot compatible :ref:`radio control system<common-rc-systems>` can be employed.

Configuration
=============

The mapping of which incoming rc channel controls which axis/directional control is fixed when GCS only control is used, However, these can be remapped using:

- :ref:`RCMAP_ROLL<RCMAP_ROLL>`
- :ref:`RCMAP_PITCHL<RCMAP_PITCH>`
- :ref:`RCMAP_YAW<RCMAP_YAW>`
- :ref:`RCMAP_THROTTLE<RCMAP_THROTTLE>`
- :ref:`RCMAP_FORWARD<RCMAP_FORWARD>`
- :ref:`RCMAP_LATERAL<RCMAP_LATERAL>`

* If a flight mode switch is used, it can be setup as following :ref:`common-rc-transmitter-flight-mode-configuration`.

* In place of the Sub ``BTNx`` functions activated from the GCS, :ref:`common-auxiliary-functions` can be setup, controlled via RC transmitter switches.

Mandatory Calibration
=====================
Once the rc receiver is attached to the autopilot and configured as described in :ref:`common-rc-systems` it must be calibrated:

- :ref:`common-radio-control-calibration`

.. toctree::
    :hidden:

    common-rc-transmitter-flight-mode-configuration
