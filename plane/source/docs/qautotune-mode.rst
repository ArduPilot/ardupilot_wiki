.. _qautotune-mode:

=========
QAUTOTUNE
=========

As of Plane-4.0, autotuning PIDs in QuadPlane is supported with this flight mode. This uses the same system as the AUTOTUNE mode for copter. It should allow you to improve the tune without having to manually adjust PIDs. It is not recommended to use QAUTOTUNE for Tailsitter pitch or yaw axis, or any axis which requires feed forward pid contributions.

Configuration
=============

- Set QAUTOTUNE mode (mode 22) on one of your mode switch positions. It may not show up as an available mode in your GCS, so you may need to set the ``FLTMODEn`` parameter in the full parameter list
- Select which axes you want to tune with :ref:`Q_AUTOTUNE_AXES<Q_AUTOTUNE_AXES>` .

Operation
=========

.. note:: QAUTOTUNE mode cannot be entered while disarmed, unlike many other QuadPlane modes.

As a QAUTOTUNE can take quite a long time (3 to 5 minutes per axis is common) you may wish to tune one axis at a time. Choose whether you want position hold support while tuning. If you enter QAUTOTUNE mode from QLOITER then it will use a loose position hold, preventing wind from carrying it away while tuning. If you start from QHOVER then it will not hold position, and you may need to reposition while tuning, but it will only do tuning (“twitching” to measure responses) while the sticks are centered.

You may need to increase the deadzone (``RCx_DZ``) on your input channels or re-calibrate your RC channels to ensure it can start the tune. Once you start the tune it will “twitch” on the axis it is tuning. So if it is tuning roll, then it does small sharp roll movements to measure response.

You can tell that QAUTOTUNE has finished when it stops twitching. You need to then land while staying in QAUTOTUNE mode and disarm without changing modes to save the tune. If you land in QAUTOTUNE and disarm after some axes have completed ,but the whole tune is not complete, then the PIDs for the axes that have completed will be saved.

You can reposition while tuning by moving your sticks. Once you re-center the sticks the tune will continue. If you change modes then the tune stops immediately.

You can read more about AutoTune for copters here: :ref:`autotune` . The autotune for quadplanes works the same way.

.. note:: Like Copter, entering QAUTOTUNE from QLOITER will attempt a loose position hold.

.. warning::QAUTOTUNE does not work on axes that need feed-forward. This means it doesn’t work on the pitch or yaw axis in tailsitters. If you want to QAUTOTUNE a tailsitter, only do the roll axis. If you do use QAUTOTUNE on the pitch or yaw axis of a tailsitter then you will end up with a very bad (possibly completely unflyable) tune.
