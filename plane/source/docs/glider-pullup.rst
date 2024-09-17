.. _glider-pullup:

========================================
Glider Pullup from Dive to Normal Flight
========================================

Intended to control and automate the pullup of a glider from a free fall dive to normal flight in hihg altitude balloon drops, this feature is not normally included in the firmware (except SITL sims) and a version built using the `Custom Firmware Build Server <https://custom.ardupilot.org>`__.

The feature is enabled with the :ref:`PUP_ENABLE <PUP_ENABLE>` parameter. When enabled it adds a new stage to the NAV_ALTITUDE_WAIT mission item, controlled by the PUP_* parameters. This stage controls the pullup maneuver where we do a g-force limited pullup out of a dive, bring the aircraft to a normal flight regime where normal mission commands can take over.

The following parameters apply to this feature:

- :ref:`PUP_ENABLE <PUP_ENABLE>`
- :ref:`PUP_ELEV_OFS <PUP_ELEV_OFS>`
- :ref:`PUP_NG_LIM <PUP_NG_LIM>`
- :ref:`PUP_NG_JERK_LIM <PUP_NG_JERK_LIM>`
- :ref:`PUP_PITCH <PUP_PITCH>`
- :ref:`PUP_ARSPD_START <PUP_ARSPD_START>`
- :ref:`PUP_PITCH_START <PUP_PITCH_START>`