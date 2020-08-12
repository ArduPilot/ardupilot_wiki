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
2                                       Ignores receiver failsafes
3                                       Add delay bytes in FPort protocol, needed by some systems, see :ref:`FPort Setup<common-FPort-receivers>`
=================================       =========

for example, to set this option to ignore receiver failsafes, you would set bit 2, or a value of "8".