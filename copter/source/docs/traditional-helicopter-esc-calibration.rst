.. _traditional-helicopter-esc-calibration:

========================================
Traditional Helicopter — ESC Calibration
========================================

This page covers specific details about calibrating the ESC for a helicopter.  If using the ESC's internal governor, ESC calibration is not required.


.. warning:: be sure to remove all blades when doing ESC calibration.

Some ESCs must be calibrated to the throttle range (ie HeliRSC output range, which defaults to 1000 to 2000us). In addition, it is often required to change ESC settings, such as enabling the governor mode and/or setting voltage protection levels.

In order to do this, you must be able to directly control the input to the ESC. By default the output function where it is attached is set to HeliRSC (:ref:`SERVO8_FUNCTION<SERVO8_FUNCTION>` = 31). In order to pass the throttle stick directly to the ESC for ESC programming per the ESC's instructions, temporarily change this to :ref:`SERVO8_FUNCTION<SERVO8_FUNCTION>` = 53. Remember to change it back to "31", after completing the ESC programming.

In addition, check to see that :ref:`RC3_MIN<RC3_MIN>` and :ref:`RC3_MAX<RC3_MAX>` match the MIN and MAX range of the HeliRSC output, which defaults to SERVO8 output (:ref:`SERVO8_MIN<SERVO8_MIN>`, :ref:`SERVO8_MAX<SERVO8_MAX>`), since passing through the throttle input will be direct and ignore those values, hich you are trying to match in the calibration. If not, temporarily change them to  match and then return them to the values when :ref:`common-radio-control-calibration` was done.
