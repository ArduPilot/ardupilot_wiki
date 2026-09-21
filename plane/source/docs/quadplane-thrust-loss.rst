.. _quadplane-thrust-loss:


VTOL Motor Thrust Loss Detection
=================================

If you see a "Potential VTOL Thrust Loss (N)" message on the GCS or in the dataflash logs, ArduPilot has detected conditions that suggest a VTOL motor may have failed, where N is the motor number identified as the likely cause.

The detection looks for a situation where the vehicle is descending while at or near full available throttle, with small attitude demands and good attitude tracking. This combination — full power, nearly level, but still sinking — is a strong indicator that a motor has lost thrust. The conditions must persist for one second before a warning is issued.

If these warnings are seen in normal hover or relaxed flight, a hardware problem should be investigated — check :ref:`motor alignment <quadplane-motor-alignment>` and motor/ESC health. If only seen in aggressive maneuvers or heavy-lift conditions, the thrust-to-weight ratio may be marginal.

The :ref:`Q_THRST_LOSS_OPT <Q_THRST_LOSS_OPT>` parameter controls when detection is active:

- Default (0) — detection runs in all flight modes where VTOL motors are active.
- Bit 0 set — detection disabled completely.
- Bit 1 set — detection only runs in pure VTOL modes; disabled during transitions and fixed-wing flight.

.. note:: Setting Bit 1 is recommended if nuisance warnings are observed during transitions, where high throttle and a brief descent can occasionally satisfy the detection conditions without a motor having actually failed.
