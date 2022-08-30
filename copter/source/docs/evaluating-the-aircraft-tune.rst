.. _evaluating-the-aircraft-tune:

============================
Evaluating the aircraft tune
============================


Most pilots will look to move to Autotune as quickly as possible once their aircraft can hover safely in AltHold. Before Autotune is run the pilot should ensure that the current tune is good enough to recover from the repeated tests run by Autotune. To test the current state of tune:

1. Take off in AltHold or STABILIZE
2. Apply small roll and pitch inputs. Start with 5 degree inputs and releasing the stick to centre, pitch, left, right, roll forward back, then all 4 points on the diagonal
3. Increase inputs gradually to full stick deflection
4. Go to full stick deflection and letting the sticks spring back to centre

If the aircraft begins to overshoot significantly or oscillate after the stick input, halt the tests before the situation begins to endanger the aircraft. The aircraft may require manual tuning (:ref:`see next section <ac_rollpitchtuning>`) before autotune can be run.

To test the stabilization loops independent of the input shaping, set the parameter: :ref:`ATC_RATE_FF_ENAB <ATC_RATE_FF_ENAB>` to 0.

1. Take off in AltHold or STABILIZE
2. Hold a roll or pitch input
3. Release the stick and observe the overshoot as the aircraft levels itself
4. Gradually increase the stick deflection to 100%

Halt the tests if the aircraft overshoots level significantly or if the aircraft oscillates, the aircraft may require manual tuning (:ref:`see next section <ac_rollpitchtuning>`) before autotune can be run.

Set :ref:`ATC_RATE_FF_ENAB <ATC_RATE_FF_ENAB>` to 1 after the tests are complete.