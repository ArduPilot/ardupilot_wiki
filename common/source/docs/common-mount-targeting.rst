.. _common-mount-targeting:


===========================
Camera Mount Mode/Targeting
===========================

The camera/gimbal direction can be controlled by the pilot using RC control(RC Targeting) if RC channels for control have been assigned (default on startup unless changed), by the autopilot during missions using the DO_SET_ROI or DO_MNT_CONTROL commands (GPS and MAVLink Targeting), not at all (just stabilizing and set to a given angle on the axes, called NEUTRAL), or when RETRACTED if a retractable mount is used to rotate the camera as it retracts for clearance.

If a retractable mount is employed, the overall mount may be deployed or retracted using an output assigned with ``SERVOx_FUNCTION`` set to "MountOpen". This will be automatically controlled by the autopilot as if it were landing gear (see :ref:`common-landing-gear`), or by pilot using an RC channel whose ``RCx_OPTION`` is set to "Landing Gear".

The default targeting mode for the first camera/gimbal is set by the :ref:`MNT1_DEFLT_MODE<MNT1_DEFLT_MODE>` parameter, while :ref:`MNT2_DEFLT_MODE<MNT2_DEFLT_MODE>` is used for the second mount, if used.

The direction the axes are set for the NEUTRAL and RETRACTED modes are set by (shown for the first mount):

- :ref:`MNT1_NEUTRAL_X<MNT1_NEUTRAL_X>`
- :ref:`MNT1_NEUTRAL_Y<MNT1_NEUTRAL_Y>`
- :ref:`MNT1_NEUTRAL_Z<MNT1_NEUTRAL_Z>`
- :ref:`MNT1_RETRACT_X<MNT1_RETRACT_X>`
- :ref:`MNT1_RETRACT_Y<MNT1_RETRACT_Y>`
- :ref:`MNT1_RETRACT_Z<MNT1_RETRACT_Z>`