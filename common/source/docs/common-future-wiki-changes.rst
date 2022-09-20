.. _common-future-wiki-changes:

==========================================
Future Wiki Changes (in Master branch now)
==========================================


[copywiki destination="plane,copter,rover,blimp"]

Add to :ref:`common-rcoutput-mapping`
=====================================

add to Generic Functions Table:

+--------------------------------+----+---------------------------------------+
|       Function                 | ID |        Available in:                  |
+--------------------------------+----+---------------------------------------+
|      RCIN1Scaled               |140 |        Plane, Copter, Rover           |
+--------------------------------+----+---------------------------------------+
|      RCIN2Scaled               |141 |        Plane, Copter, Rover           |
+--------------------------------+----+---------------------------------------+
|      RCIN3Scaled               |142 |        Plane, Copter, Rover           |
+--------------------------------+----+---------------------------------------+
|      RCIN4Scaled               |143 |        Plane, Copter, Rover           |
+--------------------------------+----+---------------------------------------+
|      RCIN5Scaled               |144 |        Plane, Copter, Rover           |
+--------------------------------+----+---------------------------------------+
|      RCIN6Scaled               |145 |        Plane, Copter, Rover           |
+--------------------------------+----+---------------------------------------+
|      RCIN7Scaled               |146 |        Plane, Copter, Rover           |
+--------------------------------+----+---------------------------------------+
|      RCIN8Scaled               |147 |        Plane, Copter, Rover           |
+--------------------------------+----+---------------------------------------+
|      RCIN9Scaled               |148 |        Plane, Copter, Rover           |
+--------------------------------+----+---------------------------------------+
|      RCIN10Scaled              |149 |        Plane, Copter, Rover           |
+--------------------------------+----+---------------------------------------+
|      RCIN11Scaled              |150 |        Plane, Copter, Rover           |
+--------------------------------+----+---------------------------------------+
|      RCIN12Scaled              |151 |        Plane, Copter, Rover           |
+--------------------------------+----+---------------------------------------+
|      RCIN13Scaled              |152 |        Plane, Copter, Rover           |
+--------------------------------+----+---------------------------------------+
|      RCIN14Scaled              |153 |        Plane, Copter, Rover           |
+--------------------------------+----+---------------------------------------+
|      RCIN15Scaled              |154 |        Plane, Copter, Rover           |
+--------------------------------+----+---------------------------------------+
|      RCIN16Scaled              |155 |        Plane, Copter, Rover           |
+--------------------------------+----+---------------------------------------+

RCIN1Scaled to RCIN16Scaled
+++++++++++++++++++++++++++

This operates similar to RCPassThru1 to RCPassThru16 above. However, instead of exactly passing the received PWM to the output, its is scaled.The RC input's dead-zone(DZ) is also obeyed.

The upper PWM range from the input trim value to its maximum input is translated to its corresponding output's trim to maximum parameter values range, and similarly for the ranges below the input's trim value as shown below:

.. image:: ../../../images/rcscaled-io.jpg
   :target: ../../_images/rcscaled-io.jpg





