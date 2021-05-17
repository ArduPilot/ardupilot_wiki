.. _common-crsf-telemetry:

=======================
TBS Crossfire Telemetry
=======================


TBS CRSF Receivers incorporate telemetry along with RC control information in their interface to ArduPilot. ArduPilot supports native CRSF telemetry and extensions to it that allow using the :Ref:`common-frsky-yaapu`. See :ref:`common-tbs-rc` for connection and setup information.

OpenTx will discover the native CRSF Telemetry sensors, which then can be displayed on `OpenTX <https://www.open-tx.org/>`_ telemetry screens or repeated from the CRSF TX module's WIFI to MAVLink Ground Control Stations:

.. image:: ../../../images/crossfire-telemetry-meaning.jpg

In addition, by setting :ref:`RC_OPTIONS<RC_OPTIONS>` bit 8,
additional ArduPilot telemetry items are transferred which allows use of the :ref:`common-frsky-yaapu` on OpenTX transmitters. Limitations and additional information is shown `here <https://discuss.ardupilot.org/t/passthrough-telemetry-over-crsf-crossfire>`__.

Finally, several OpenTX scripts are normally provided for adjusting CRSF system parameters as well as providing ArduPilot parameter adjustment, similar in function to ArduPilot's :ref:`common-paramosd` feature.

