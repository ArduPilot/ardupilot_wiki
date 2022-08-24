.. _common_rc_options:

==========
RC Options
==========


:ref:`RC_OPTIONS<RC_OPTIONS>` bitmask provides several options related to how RC inputs and overrides by the Ground Control Station (GCS) are handled, depending on which bits are set ("1"):

=================================       =========
:ref:`RC_OPTIONS<RC_OPTIONS>` bit       Function
=================================       =========
0                                       Ignores any attached RC receiver outputs
1                                       Ignores any RC overrides received from the GCS
2                                       Ignores receiver failsafe bit
3                                       Add delay bytes in FPort protocol, needed by some systems, see :ref:`FPort Setup<common-FPort-receivers>`
4                                       Log RC raw RC input bytes for serial protocols
5                                       Require Throttle input at idle position in order to arm
6                                       Allows arming if the rudder,elevator, or aileron
                                        stick is not neutral
7                                       Allow Aux Switches to honor the ``RCx_REVERSED`` parameter
8                                       Use Passthru extensions for CRSF telemetry (see :ref:`common-frsky-passthrough`)
9                                       Suppress CRSF mode/rate messages for ELRS systems
10                                      Enable multiple receiver support on autopilot
=================================       =========

for example, to set this option to ignore receiver failsafe bits, you would set bit 2, or a value of "4" (2^2=4). This may be usefull when using ground station control beyond the range of the RC system which can set its receiver's outputs to trim values upon RC signal loss, but still has a failsafe bit in the protocol which would otherwise force an RC failsafe to occur.


There is also an :ref:`RC_PROTOCOLS<RC_PROTOCOLS>` bitmask that can be used to restrict which RC protocols are detected and used. This is useful in cases where the RC protocol autodetection fails and an incorrect RC protocol handler is chosen. This is rare, but if you do find it happens then you can lock in a single RC protocol that can be detected and used with this parameter.

.. note:: Once ArduPilot decodes the RC protocol being used, it will send a message declaring which protocol is being decoded to the Ground Control Station.
