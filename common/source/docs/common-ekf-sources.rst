.. _common-ekf-sources:

==================================
EKF Source Selection and Switching
==================================

The EKF position and velocity sources used for its internal updates can be set using the ``EKx_SRCn_y`` parameters. For EKF3's primary sources, these are:

- :ref:`EK3_SRC1_POSXY<EK3_SRC1_POSXY>`: Position Horizontal Source
- :ref:`EK3_SRC1_VELXY<EK3_SRC1_VELXY>`: Velocity Horizontal Source
- :ref:`EK3_SRC1_POSZ<EK3_SRC1_POSZ>`: Position Vertical Source
- :ref:`EK3_SRC1_VELZ<EK3_SRC1_VELZ>`: Velocity Vertical Source
- :ref:`EK3_SRC1_YAW<EK3_SRC1_YAW>`: Yaw Source

The options for these are generally self-explanatory. However, the :ref:`EK3_SRC1_YAW<EK3_SRC1_YAW>`  source options need a bit more explanation:

- Option 1: Compass is the normal default.
- Option 2: GPS is used with GPS that can supply yaw (see :ref:`common-gps-for-yaw`)
- Option 3: GPS with compass fallback is the same as GPS option, but while it has GPS yaw available it learns compass offsets to make the compass yaw match the GPS yaw. This means that if you lose GPS yaw while flying then the compass is ready to takeover immediately, with a smooth change from GPS yaw to compass yaw.
- Option 6: ExternalNav is used when a companion device is providing yaw information
- Option 8: GSF (Gaussian Sum Filter) provides yaw information based on GPS velocity and position changes and Plane will automatically use this in :ref:`compass-less <common-compassless>` configurations,even if not selected. Copter requires this to be set explicitly in these configurations.

Source Switching
================

Three set of EKF position and velocity source parameters are provided which can be selected via an :ref:`RC switch <common-auxiliary-functions>` set to "90". Otherwise, the SRC1 set is used.

This feature is especially helpful when using :ref:`common-non-gps-to-gps`.


